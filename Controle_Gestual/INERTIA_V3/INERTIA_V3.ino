/*
 * Controle BLE V3 — ESP32-S3 + MPU6050 + gatilho
 *
 * Sem Wi-Fi e sem WebServer. O controle pode conectar ao carrinho enquanto
 * permanece travado; nenhum comando de movimento é transmitido até UNLOCK.
 * Ajustes locais são feitos pelo monitor serial USB.
 *
 * Pinos:
 * BTN_FRONT=2, BTN_BACK=3, LED_R=4, LED_G=5, LED_B=6,
 * SDA=8, SCL=9, BTN_LOCK=10, BUZZER=48.
 *
 * Botão LOCK:
 * - clique simples: recalibra os ângulos em posição neutra;
 * - duplo clique: alterna LOCK/NORMAL;
 * - segurar 5 s: entra ou sai do CONFIG BLE;
 * - segurar 10 s: limpa o vínculo e inicia novo bind.
 *
 * Console serial 115200:
 * HELP | STATUS | LOCK | UNLOCK | CAL | REBIND | ANGLES?
 * ANGLES=pitchHigh,rollLow,rollMid,rollHigh
 * PWM? | PWM=min,med,max
 *
 * BLE: serviço FFE0, característica FFE1. O padrão usa frames V3:
 * @D,V=2,R=0,S=1*CRC8;
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

// ---------------- Pinagem ----------------
const uint8_t BTN_FRONT = 2, BTN_BACK = 3, BTN_LOCK = 10;
const uint8_t LED_R = 4, LED_G = 5, LED_B = 6, BUZZER = 48;
const uint8_t SDA_PIN = 8, SCL_PIN = 9;

// Inversão dos sentidos lógicos da IMU. Altere para true se o movimento
// correspondente estiver invertido no carrinho. A calibração continua válida.
const bool INVERT_PITCH = false;
const bool INVERT_ROLL = false;

// Brilho máximo dos LEDs em PWM: 0 = apagado, 255 = brilho total.
// Valores entre 45 e 100 costumam ser confortáveis em LEDs fortes.
const uint8_t LED_MAX_BRIGHTNESS = 50;

// ---------------- BLE ----------------
static BLEUUID SERVICE_UUID((uint16_t)0xFFE0);
static BLEUUID DATA_UUID((uint16_t)0xFFE1);
static BLEUUID CONFIG_SERVICE_UUID("7b7e0001-6f4a-4c32-9c51-5a7f3a000001");
static BLEUUID CONFIG_CHAR_UUID("7b7e0001-6f4a-4c32-9c51-5a7f3a000002");
const char *TARGET_PREFIX = "CARRINHO";
const bool USE_V3_FRAMES = true;
const uint32_t BLE_RETRY_MS = 2500;
const uint32_t HEARTBEAT_MS = 80;

// ---------------- MPU6050 ----------------
const uint8_t MPU_ADDR = 0x68;
const uint8_t WHO_AM_I = 0x75, PWR_MGMT_1 = 0x6B, ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_XOUT_H = 0x3B;

Preferences prefs;
BLEClient *client = nullptr;
BLERemoteCharacteristic *dataChar = nullptr;
BLEAdvertisedDevice *candidate = nullptr;
BLEServer *configServer = nullptr;
BLECharacteristic *configChar = nullptr;
bool configBleActive = false;

volatile bool connectRequested = false;
volatile bool rescanRequested = false;
bool scanCallbackInstalled = false;
bool bleConnected = false;
String boundMac;
String deviceName;
uint32_t retryAt = 0;
uint32_t lastHeartbeat = 0;
uint8_t sequenceNo = 0;

// ---------------- Estado ----------------
enum ControlMode : uint8_t { MODE_LOCK, MODE_NORMAL, MODE_CONFIG };
ControlMode mode = MODE_LOCK;
float pitch = 0, roll = 0, pitchOffset = 0, rollOffset = 0;
float pitchRaw = 0, rollRaw = 0;
float pitchHigh = 20, rollLow = 10, rollMid = 20, rollHigh = 30;
uint8_t pwmMin = 70, pwmMid = 130, pwmMax = 255;
int8_t velocity = 0, rotation = 0;
uint32_t lastImuRead = 0;

// ---------------- Botão ----------------
bool previousButton = HIGH;
uint32_t pressedAt = 0, lastClickAt = 0;
bool holdActionDone = false, rebindHoldDone = false;
uint8_t clickCount = 0;

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  // LED RGB de ânodo comum.
  uint8_t limitedR = (uint16_t)r * LED_MAX_BRIGHTNESS / 255;
  uint8_t limitedG = (uint16_t)g * LED_MAX_BRIGHTNESS / 255;
  uint8_t limitedB = (uint16_t)b * LED_MAX_BRIGHTNESS / 255;
  analogWrite(LED_R, 255 - limitedR); analogWrite(LED_G, 255 - limitedG); analogWrite(LED_B, 255 - limitedB);
}
void ledOff() { setLed(0, 0, 0); }
void beep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(BUZZER, HIGH); delay(35); digitalWrite(BUZZER, LOW); delay(35);
  }
}
void updateLed() {
  if (mode == MODE_CONFIG) { setLed(0, 150, 150); return; }
  if (mode == MODE_LOCK) { setLed(bleConnected ? 255 : 180, 0, 0); return; }
  if (bleConnected) { setLed(0, 255, 0); return; }
  uint8_t v = (millis() / 8) % 255;
  setLed(0, 0, v < 128 ? v * 2 : (255 - v) * 2);
}

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1); return Wire.available() ? Wire.read() : 0;
}
void readMpu() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(ACCEL_XOUT_H); Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read();
  float fax = ax, fay = ay, faz = az;
  // A IMU está montada com os eixos lógicos trocados na estrutura:
  // o eixo calculado originalmente como roll deve controlar pitch,
  // e o eixo calculado originalmente como pitch deve controlar roll.
  float physicalPitch = atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0f / PI;
  float physicalRoll = atan2(fax, sqrt(fay * fay + faz * faz)) * 180.0f / PI;
  // Mantém a orientação física já ajustada: physicalRoll -> pitch e
  // physicalPitch -> roll. Os valores raw permitem zerar sem acumular offset.
  pitchRaw = INVERT_PITCH ? -physicalRoll : physicalRoll;
  rollRaw = INVERT_ROLL ? physicalPitch : -physicalPitch;
  pitch = pitchRaw - pitchOffset;
  roll = rollRaw - rollOffset;
}
void calibrateAngles() {
  float pitchSum = 0, rollSum = 0;
  for (uint8_t sample = 0; sample < 20; sample++) {
    readMpu();
    pitchSum += pitchRaw;
    rollSum += rollRaw;
    delay(5);
  }
  // Zerar significa capturar a posição atual, não somar a posição atual
  // ao offset anterior. A soma fazia a calibração degradar a cada clique.
  pitchOffset = pitchSum / 20.0f;
  rollOffset = rollSum / 20.0f;
  pitch = 0;
  roll = 0;
  prefs.begin("angles", false); prefs.putFloat("pitchOff", pitchOffset); prefs.putFloat("rollOff", rollOffset); prefs.end();
  Serial.printf("[CAL] pitchOffset=%.2f rollOffset=%.2f\n", pitchOffset, rollOffset); beep(1);
}

void loadSettings() {
  prefs.begin("angles", true);
  pitchHigh = prefs.getFloat("pitchHigh", 20); rollLow = prefs.getFloat("rollLow", 10);
  rollMid = prefs.getFloat("rollMid", 20); rollHigh = prefs.getFloat("rollHigh", 30);
  pitchOffset = prefs.getFloat("pitchOff", 0); rollOffset = prefs.getFloat("rollOff", 0); prefs.end();
  prefs.begin("pwm", true); pwmMin = prefs.getUChar("min", 70); pwmMid = prefs.getUChar("mid", 130); pwmMax = prefs.getUChar("max", 255); prefs.end();
}
void saveAngles() {
  prefs.begin("angles", false); prefs.putFloat("pitchHigh", pitchHigh); prefs.putFloat("rollLow", rollLow); prefs.putFloat("rollMid", rollMid); prefs.putFloat("rollHigh", rollHigh); prefs.end();
}
void savePwm() {
  prefs.begin("pwm", false); prefs.putUChar("min", pwmMin); prefs.putUChar("mid", pwmMid); prefs.putUChar("max", pwmMax); prefs.end();
}

uint8_t crc8(const String &payload) { uint16_t sum = 0; for (size_t i = 0; i < payload.length(); i++) sum += (uint8_t)payload[i]; return (uint8_t)sum; }
String makeFrame(const String &payload) {
  char crc[3]; snprintf(crc, sizeof(crc), "%02X", crc8(payload)); return "@" + payload + "*" + crc + ";";
}
void sendBle(const String &payload) {
  if (!bleConnected || dataChar == nullptr) return;
  String out = USE_V3_FRAMES ? makeFrame(payload) : payload;
  dataChar->writeValue((uint8_t *)out.c_str(), out.length(), true);
}

void sendMotionStop() {
  velocity = 0;
  rotation = 0;
  sequenceNo++;
  // O frame de movimento zerado elimina qualquer comando R/L persistente
  // no receptor; STOP é mantido como compatibilidade com o firmware antigo.
  sendBle("D,V=0,R=0,S=" + String(sequenceNo));
  sendBle("STOP");
}

void stopConfigBle();

void sendConfigReply(const String &text) {
  if (configChar == nullptr) return;
  configChar->setValue(text.c_str());
  configChar->notify();
}

void processConfigCommand(String line) {
  line.trim(); line.toUpperCase();
  if (line == "STATUS") {
    sendConfigReply("STATUS,MODE=CONFIG,BLE=" + String(bleConnected ? 1 : 0));
  } else if (line == "CAL") {
    calibrateAngles(); sendConfigReply("OK,CAL");
  } else if (line == "ANGLES?") {
    sendConfigReply("ANGLES=" + String(pitchHigh, 1) + "," + String(rollLow, 1) + "," + String(rollMid, 1) + "," + String(rollHigh, 1));
  } else if (line.startsWith("ANGLES=")) {
    float a, b, c, d;
    if (sscanf(line.c_str() + 7, "%f,%f,%f,%f", &a, &b, &c, &d) == 4 && a > 0 && b > 0 && b <= c && c <= d && d <= 90) {
      pitchHigh = a; rollLow = b; rollMid = c; rollHigh = d; saveAngles(); sendConfigReply("OK,ANGLES");
    } else sendConfigReply("ERR,ANGLES");
  } else if (line == "PWM?") {
    sendConfigReply("PWM=" + String(pwmMin) + "," + String(pwmMid) + "," + String(pwmMax));
  } else if (line.startsWith("PWM=")) {
    int a, b, c;
    if (sscanf(line.c_str() + 4, "%d,%d,%d", &a, &b, &c) == 3 && a >= 0 && a <= b && b <= c && c <= 255) {
      pwmMin = a; pwmMid = b; pwmMax = c; savePwm(); sendConfigReply("OK,PWM");
    } else sendConfigReply("ERR,PWM");
  } else if (line == "EXIT") {
    sendConfigReply("OK,EXIT"); stopConfigBle(); mode = MODE_LOCK;
  } else sendConfigReply("ERR,COMMAND");
}

class ConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String command = characteristic->getValue().c_str();
    if (mode == MODE_CONFIG) processConfigCommand(command);
  }
};

void stopConfigBle() {
  if (!configBleActive) return;
  BLEDevice::getAdvertising()->stop();
  configBleActive = false;
  Serial.println("[CFG] BLE de configuração encerrado");
}

void startConfigBle() {
  if (configBleActive) return;
  if (configServer == nullptr) configServer = BLEDevice::createServer();
  BLEService *service = configServer->createService(CONFIG_SERVICE_UUID);
  configChar = service->createCharacteristic(CONFIG_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  configChar->addDescriptor(new BLE2902());
  configChar->setCallbacks(new ConfigCallbacks());
  configChar->setValue("READY");
  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(CONFIG_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  configBleActive = true;
  Serial.println("[CFG] BLE ativo: CONTROLE-CFG");
}

void toggleConfigMode() {
  if (mode == MODE_CONFIG) {
    sendMotionStop(); stopConfigBle(); mode = MODE_LOCK; beep(2);
    Serial.println("[CFG] saindo; controle permanece LOCK");
  } else {
    sendMotionStop(); mode = MODE_CONFIG; startConfigBle(); beep(1);
    Serial.println("[CFG] modo de configuração BLE");
  }
}

void calculateCommand() {
  bool front = digitalRead(BTN_FRONT) == LOW, back = digitalRead(BTN_BACK) == LOW;
  velocity = 0; rotation = 0;
  if (front != back && mode == MODE_NORMAL) {
    if (front) velocity = pitch > pitchHigh ? 1 : pitch < -pitchHigh ? 3 : 2;
    else velocity = pitch < -pitchHigh ? -1 : pitch > pitchHigh ? -3 : -2;
  }
  if (mode == MODE_NORMAL) {
    if (roll > rollHigh) rotation = 3; else if (roll > rollMid) rotation = 2; else if (roll > rollLow) rotation = 1;
    else if (roll < -rollHigh) rotation = -3; else if (roll < -rollMid) rotation = -2; else if (roll < -rollLow) rotation = -1;
  }
}
void sendCurrentCommand() {
  if (USE_V3_FRAMES) {
    sequenceNo++;
    sendBle("D,V=" + String(velocity) + ",R=" + String(rotation) + ",S=" + String(sequenceNo));
  } else {
    String v = velocity == 0 ? "V0" : velocity > 0 ? "VF" + String(velocity) : "VB" + String(-velocity);
    String r = rotation == 0 ? "0" : rotation > 0 ? "R" + String(rotation) : "L" + String(-rotation);
    sendBle(v + "," + r);
  }
}

class AdvertisedCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) override {
    if (!device.haveName()) return;
    String name = device.getName().c_str();
    if (!name.startsWith(TARGET_PREFIX)) return;
    BLEDevice::getScan()->stop();
    if (candidate) delete candidate;
    candidate = new BLEAdvertisedDevice(device);
    deviceName = name; connectRequested = true;
  }
};
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *) override { bleConnected = true; retryAt = 0; Serial.println("[BLE] conectado"); beep(2); }
  void onDisconnect(BLEClient *) override { bleConnected = false; dataChar = nullptr; Serial.println("[BLE] desconectado"); rescanRequested = true; }
};
void startScan() {
  BLEScan *scan = BLEDevice::getScan();
  if (!scanCallbackInstalled) { scan->setAdvertisedDeviceCallbacks(new AdvertisedCallbacks()); scan->setActiveScan(true); scan->setInterval(80); scan->setWindow(50); scanCallbackInstalled = true; }
  Serial.println("[BLE] procurando CARRINHO..."); scan->start(4, false);
}
void clearBind() {
  prefs.begin("ble", false); prefs.remove("mac"); prefs.end(); boundMac = "";
}
void saveBind(const String &mac) { prefs.begin("ble", false); prefs.putString("mac", mac); prefs.end(); boundMac = mac; }
void disconnectBle() { if (client && client->isConnected()) client->disconnect(); bleConnected = false; dataChar = nullptr; }
void rebind() {
  sendMotionStop(); stopConfigBle(); disconnectBle(); clearBind(); mode = MODE_LOCK; velocity = rotation = 0; connectRequested = false; rescanRequested = true; retryAt = millis() + 300; beep(3); Serial.println("[BIND] vínculo apagado; novo scan será iniciado");
}
void loadBind() { prefs.begin("ble", true); boundMac = prefs.getString("mac", ""); prefs.end(); }

bool connectTarget() {
  if (!client) { client = BLEDevice::createClient(); client->setClientCallbacks(new ClientCallbacks()); }
  bool ok = false;
  if (candidate) ok = client->connect(candidate);
  else if (boundMac.length()) ok = client->connect(BLEAddress(boundMac.c_str()));
  if (!ok) { Serial.println("[BLE] conexão falhou"); return false; }
  BLERemoteService *service = client->getService(SERVICE_UUID);
  if (!service) { client->disconnect(); return false; }
  dataChar = service->getCharacteristic(DATA_UUID);
  if (!dataChar || !dataChar->canWrite()) { client->disconnect(); dataChar = nullptr; return false; }
  if (!boundMac.length() && candidate) saveBind(candidate->getAddress().toString().c_str());
  if (dataChar->canNotify()) dataChar->registerForNotify([](BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) { Serial.printf("[RX] %.*s\n", (int)len, data); });
  sendBle("STOP"); return true;
}

void toggleLock() {
  if (mode == MODE_CONFIG) return;
  mode = mode == MODE_LOCK ? MODE_NORMAL : MODE_LOCK;
  if (mode == MODE_LOCK) sendMotionStop(); else { velocity = 0; rotation = 0; }
  Serial.println(mode == MODE_LOCK ? "[MODE] LOCK" : "[MODE] NORMAL"); beep(mode == MODE_LOCK ? 2 : 1);
}
void handleLockButton() {
  bool current = digitalRead(BTN_LOCK); uint32_t now = millis();
  if (previousButton == HIGH && current == LOW) { pressedAt = now; holdActionDone = false; rebindHoldDone = false; }
  if (current == LOW && !holdActionDone && now - pressedAt >= 5000) { holdActionDone = true; toggleConfigMode(); }
  if (current == LOW && !rebindHoldDone && now - pressedAt >= 10000) { rebindHoldDone = true; rebind(); }
  if (previousButton == LOW && current == HIGH && !holdActionDone) {
    if (now - lastClickAt < 450) { clickCount = 0; toggleLock(); }
    else { clickCount = 1; lastClickAt = now; }
  }
  if (clickCount == 1 && now - lastClickAt > 450) { clickCount = 0; if (mode != MODE_CONFIG) calibrateAngles(); }
  previousButton = current;
}

void printHelp() {
  Serial.println("HELP STATUS LOCK UNLOCK CAL REBIND ANGLES? ANGLES=a,b,c,d PWM? PWM=min,mid,max");
}
void processSerial(String line) {
  line.trim(); line.toUpperCase();
  if (line == "HELP") printHelp(); else if (line == "STATUS") Serial.printf("mode=%s ble=%d name=%s mac=%s pitch=%.1f roll=%.1f rawPitch=%.1f rawRoll=%.1f\n", mode == MODE_CONFIG ? "CONFIG" : mode == MODE_LOCK ? "LOCK" : "NORMAL", bleConnected, deviceName.c_str(), boundMac.c_str(), pitch, roll, pitchRaw, rollRaw);
  else if (line == "LOCK") { mode = MODE_LOCK; sendMotionStop(); }
  else if (line == "UNLOCK") { mode = MODE_NORMAL; velocity = 0; rotation = 0; sendMotionStop(); }
  else if (line == "CAL") calibrateAngles();
  else if (line == "REBIND") rebind();
  else if (line == "ANGLES?") Serial.printf("ANGLES=%.1f,%.1f,%.1f,%.1f\n", pitchHigh, rollLow, rollMid, rollHigh);
  else if (line.startsWith("ANGLES=")) { sscanf(line.c_str() + 7, "%f,%f,%f,%f", &pitchHigh, &rollLow, &rollMid, &rollHigh); saveAngles(); }
  else if (line == "PWM?") Serial.printf("PWM=%u,%u,%u\n", pwmMin, pwmMid, pwmMax);
  else if (line.startsWith("PWM=")) { int a,b,c; if (sscanf(line.c_str()+4, "%d,%d,%d", &a,&b,&c)==3 && a>=0 && a<=b && b<=c && c<=255) { pwmMin=a; pwmMid=b; pwmMax=c; savePwm(); sendBle("C,UNLOCK"); delay(100); sendBle("C,PROFILE," + String(pwmMin) + "," + String(pwmMid) + "," + String(pwmMax)); } }
  else if (line.length()) Serial.println("[CMD] desconhecido; use HELP");
}

void setup() {
  Serial.begin(115200); pinMode(BTN_FRONT, INPUT_PULLUP); pinMode(BTN_BACK, INPUT_PULLUP); pinMode(BTN_LOCK, INPUT_PULLUP); pinMode(BUZZER, OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT); ledOff();
  Wire.begin(SDA_PIN, SCL_PIN); writeReg(PWR_MGMT_1, 0); writeReg(ACCEL_CONFIG, 0x10); loadSettings(); loadBind();
  uint8_t id = readReg(WHO_AM_I); Serial.printf("[BOOT] MPU WHO_AM_I=0x%02X\n", id); printHelp();
  mode = MODE_LOCK; BLEDevice::init("ESP32_Remote_V3");
  if (boundMac.length()) connectRequested = true; else rescanRequested = true;
}
void loop() {
  uint32_t now = millis();
  if (Serial.available()) processSerial(Serial.readStringUntil('\n'));
  handleLockButton();
  if (now - lastImuRead >= 20) { lastImuRead = now; readMpu(); calculateCommand(); }
  if (connectRequested && !bleConnected) { connectRequested = false; if (!connectTarget()) { rescanRequested = true; } }
  if (rescanRequested && !bleConnected && (int32_t)(now - retryAt) >= 0) { rescanRequested = false; retryAt = now + BLE_RETRY_MS; startScan(); }
  if (bleConnected && now - lastHeartbeat >= HEARTBEAT_MS) { lastHeartbeat = now; if (mode == MODE_NORMAL) sendCurrentCommand(); else sendMotionStop(); }
  updateLed();
}
