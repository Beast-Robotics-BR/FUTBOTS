/*
 * =====================================================
 *  ESP32-S3 Super Mini — Controle Gesture + Gatilho
 *  SYSTRONYX / IFFar Robotics Platform  v2.3 (BLE Client + Web Config)
 * =====================================================
 *  Atua como CLIENTE BLE, conectando-se por PREFIXO DE
 *  NOME (ex: "CARRINHO-A", "CARRINHO-B", ...) ou por MAC
 *  vinculado, ao módulo BLE do Arduino Nano (serviço 0xFFE0).
 *
 *  NOVO EM v2.3:
 *   - Modo CONFIG agora sobe um Access Point próprio
 *     ("CONTROLE-CFG", aberto, sem senha) com uma pagina
 *     web simples pra ajustar os limiares de angulo/gatilho
 *     (PITCH_HIGH, ROLL_LOW, ROLL_MID, ROLL_HIGH) em vez de
 *     precisar editar o codigo. Valores salvos em NVS.
 *   - O MPU agora e lido sempre (mesmo em CONFIG), pra a
 *     pagina web mostrar pitch/roll ao vivo durante o ajuste.
 *   - WiFi so fica ligado durante CONFIG (que ja nao envia
 *     comandos BLE) e e desligado ao sair do modo, evitando
 *     disputa de radio com o BLE durante o uso normal.
 *
 *  DRIVER MPU6050:
 *    Acesso direto por registrador via I2C (sem Adafruit_MPU6050),
 *    aceita WHO_AM_I 0x68 ou 0x70. Acelerometro +-8g, giroscopio
 *    +-500 graus/s (giro nao usado no calculo ainda).
 *
 *  PINAGEM:
 *    GPIO 2   → Botão gatilho FRENTE (GND quando pressionado)
 *    GPIO 3   → Botão gatilho TRÁS   (GND quando pressionado)
 *    GPIO 4   → LED R (ânodo comum)
 *    GPIO 5   → LED G (ânodo comum)
 *    GPIO 6   → LED B (ânodo comum)
 *    GPIO 7   → (reservado: leitura bateria)
 *    GPIO 8   → IMU SDA
 *    GPIO 9   → IMU SCL
 *    GPIO 10  → Botão trava/config (GND quando pressionado)
 *
 *  MODOS:
 *    NORMAL   → LED verde · envia comandos via BLE para o Nano
 *    LOCK     → LED vermelho · gatilho ignorado, não envia comandos
 *    CONFIG   → LED amarelo pulsando · sem BLE · AP web ativo
 *               (conecte no WiFi "CONTROLE-CFG" e acesse o IP
 *               mostrado no Serial, ex: http://192.168.4.1)
 *
 *  BOTÃO TRAVA (GPIO 10):
 *    Clique simples   → recalibra ângulo
 *    Duplo clique     → LOCK / UNLOCK (ou sai do CONFIG se estiver nele)
 *    Segurar 5s       → entra/sai do modo CONFIG
 *    Segurar 10s      → limpa o vínculo BLE atual e re-vincula (bind)
 *
 *  PROTOCOLO BLE (para o Nano):
 *    Envia pacote: "VF2,R1" (velocidade, rotação -- sem PWM, definido
 *    hoje no proprio Nano/interface do carrinho)
 *    Recebe ACKs do Nano via notificação em 0xFFE1
 * =====================================================
 */

#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>

#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_CONFIG   0x1C
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_XOUT_H   0x3B
#define REG_WHO_AM_I       0x75

int16_t axRaw, ayRaw, azRaw;
int16_t gxRaw, gyRaw, gzRaw;

// ── Pinos ─────────────────────────────────────────
#define BTN_FRONT   2
#define BTN_BACK    3
#define LED_R_PIN   4
#define LED_G_PIN   5
#define LED_B_PIN   6
// GPIO 7 reservado bateria
#define SDA_PIN     8
#define SCL_PIN     9
#define BTN_LOCK    10
#define BUZZER_PIN  48   // ajuste se necessário

// ── Limiares de ângulo (agora ajustáveis via interface web em CONFIG) ──
#define DEFAULT_PITCH_HIGH   20.0f   // °
#define DEFAULT_ROLL_LOW     10.0f
#define DEFAULT_ROLL_MID     20.0f
#define DEFAULT_ROLL_HIGH    30.0f

float PITCH_HIGH = DEFAULT_PITCH_HIGH;
float ROLL_LOW   = DEFAULT_ROLL_LOW;
float ROLL_MID   = DEFAULT_ROLL_MID;
float ROLL_HIGH  = DEFAULT_ROLL_HIGH;

// ── Padrões PWM ───────────────────────────────────
#define DEFAULT_PWM_MIN   70
#define DEFAULT_PWM_MID  130
#define DEFAULT_PWM_MAX  255

// ── Filtro de conexao por nome ────────────────────
static const char* TARGET_NAME_PREFIX = "CARRINHO";

// ── UUIDs do Módulo BLE do Nano (confirmados por dump real) ──
static BLEUUID serviceUUID((uint16_t)0xFFE0);
static BLEUUID charDataUUID((uint16_t)0xFFE1);      // WRITE + NOTIFY (usada nos dois sentidos)
static BLEUUID charSecondaryUUID((uint16_t)0xFFE2); // WRITE apenas, sem uso definido por ora

// ── Objetos ───────────────────────────────────────
Preferences       prefs;

BLEClient*  pClient  = nullptr;
BLERemoteCharacteristic* pWriteCharacteristic = nullptr;
bool bleConnected = false;
bool scanComplete = false;
String connectedDeviceName = "";

// Padrao seguro de conexao: onResult() so guarda o device e sinaliza;
// a conexao de fato acontece no loop(), fora do contexto do callback do
// scan (chamar connect() de dentro do onResult pode travar o stack BLE).
static BLEAdvertisedDevice* pTargetDevice = nullptr;
static volatile bool doConnect  = false;
static volatile bool needRescan = false;

// Bind por MAC: se preenchido, pula o scan por nome e conecta direto.
static String boundMacStr   = "";
static bool   boundMacKnown = false;
static bool   scanCallbackSet = false;

// ── Web server de configuração (ativo só durante MODE_CONFIG) ──
const char* CONFIG_AP_SSID = "CONTROLE-CFG"; // AP aberto, sem senha
WebServer configServer(80);
bool webServerActive = false;

// ── Estado global ─────────────────────────────────
enum Mode { MODE_NORMAL, MODE_LOCK, MODE_CONFIG };
Mode currentMode = MODE_NORMAL;

float pitch = 0, roll = 0;
float pitchOffset = 0, rollOffset = 0;

String velocityCmd = "VF0";
String rotationCmd = "R0";

uint8_t pwmMin = DEFAULT_PWM_MIN;
uint8_t pwmMid = DEFAULT_PWM_MID;
uint8_t pwmMax = DEFAULT_PWM_MAX;

unsigned long lastSend    = 0;
const unsigned long SEND_INTERVAL = 10;

// ── Botão trava ───────────────────────────────────
bool     lastLockState      = HIGH;
unsigned long lockPressTime = 0;
bool     lockHoldFired      = false;
bool     lockHoldFired10    = false;   // segundo threshold (10s) = rebind

unsigned long firstClickTime     = 0;
bool          waitingSecondClick = false;

// ── Protótipos ────────────────────────────────────
void setupLED();
void updateLED();
void setLED(uint8_t r, uint8_t g, uint8_t b);
void flashLED(uint8_t r, uint8_t g, uint8_t b, int times = 1, int ms = 120);
void pulseLED_yellow();
void pulseLED_blue();

void setupMPU();
void readMPU();
void zeroAngles();

void calculateCommands();

void handleLockButton();

void setupBLE_Client();
void sendCommandToNano(String cmd);
void processAck(String ack);
void startNameScan();
bool connectToServer();
void loadBoundMac();
void saveBoundMac(String mac);
void clearBoundMac();
void rebindRequest();

void loadConfig();
void saveConfig();
void resetConfig();

void loadAngleConfig();
void saveAngleConfig();
void startConfigWebServer();
void stopConfigWebServer();
void handleWebRoot();
void handleWebSave();

void beep(int times);

void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)1);

  if (Wire.available())
    return Wire.read();

  return 0;
}

void readRawData()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  axRaw = (Wire.read()<<8) | Wire.read();
  ayRaw = (Wire.read()<<8) | Wire.read();
  azRaw = (Wire.read()<<8) | Wire.read();

  Wire.read();
  Wire.read();

  gxRaw = (Wire.read()<<8) | Wire.read();
  gyRaw = (Wire.read()<<8) | Wire.read();
  gzRaw = (Wire.read()<<8) | Wire.read();
}


// ── LED RGB (ânodo comum) ─────────────────────────
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LED_R_PIN, 255 - r);
  analogWrite(LED_G_PIN, 255 - g);
  analogWrite(LED_B_PIN, 255 - b);
}
void ledOff()     { setLED(0,   0,   0);   }
void ledRed()     { setLED(255, 0,   0);   }
void ledGreen()   { setLED(0,   255, 0);   }
void ledBlue()    { setLED(0,   0,   255); }
void ledYellow()  { setLED(255, 180, 0);   }
void ledCyan()    { setLED(0,   255, 255); }
void ledWhite()   { setLED(255, 255, 255); }

void flashLED(uint8_t r, uint8_t g, uint8_t b, int times, int ms) {
  for (int i = 0; i < times; i++) {
    setLED(r, g, b);
    delay(ms);
    ledOff();
    if (i < times - 1) delay(ms / 2);
  }
}

void pulseLED_yellow() {
  static int val = 0;
  static int dir = 5;
  static unsigned long last = 0;
  if (millis() - last < 15) return;
  last = millis();
  val += dir;
  if (val >= 200 || val <= 10) dir = -dir;
  analogWrite(LED_R_PIN, 255 - val);
  analogWrite(LED_G_PIN, 255 - (val * 70 / 100));
  analogWrite(LED_B_PIN, 255);
}

void pulseLED_blue() {
  static int val = 0;
  static int dir = 4;
  static unsigned long last = 0;
  if (millis() - last < 20) return;
  last = millis();
  val += dir;
  if (val >= 190 || val <= 5) dir = -dir;
  analogWrite(LED_R_PIN, 255);
  analogWrite(LED_G_PIN, 255);
  analogWrite(LED_B_PIN, 255 - val);
}

void setupLED() {
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  ledOff();
}

void updateLED() {
  if (currentMode == MODE_CONFIG) {
    pulseLED_yellow();
    return;
  }
  if (currentMode == MODE_LOCK) {
    ledRed();
    return;
  }
  // MODE_NORMAL
  if (!bleConnected) {
    pulseLED_blue();   // aguardando conexão
  } else {
    ledGreen();        // conectado
  }
}

// ── Buzzer ────────────────────────────────────────
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(70);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(70);
  }
}

// ── NVS: carregar / salvar / resetar (PWM) ───────
void loadConfig() {
  prefs.begin("pwmcfg", true);
  pwmMin = prefs.getUChar("min", DEFAULT_PWM_MIN);
  pwmMid = prefs.getUChar("mid", DEFAULT_PWM_MID);
  pwmMax = prefs.getUChar("max", DEFAULT_PWM_MAX);
  prefs.end();
  Serial.printf("[CFG] Carregado: min=%d mid=%d max=%d\n", pwmMin, pwmMid, pwmMax);
}

void saveConfig() {
  prefs.begin("pwmcfg", false);
  prefs.putUChar("min", pwmMin);
  prefs.putUChar("mid", pwmMid);
  prefs.putUChar("max", pwmMax);
  prefs.end();
  Serial.printf("[CFG] Salvo: min=%d mid=%d max=%d\n", pwmMin, pwmMid, pwmMax);
}

void resetConfig() {
  pwmMin = DEFAULT_PWM_MIN;
  pwmMid = DEFAULT_PWM_MID;
  pwmMax = DEFAULT_PWM_MAX;
  saveConfig();
  Serial.println("[CFG] Reset para padrões");
}

// ── NVS: limiares de ângulo (pitch/roll) ─────────
void loadAngleConfig() {
  prefs.begin("anglecfg", true);
  PITCH_HIGH = prefs.getFloat("pitchHigh", DEFAULT_PITCH_HIGH);
  ROLL_LOW   = prefs.getFloat("rollLow",   DEFAULT_ROLL_LOW);
  ROLL_MID   = prefs.getFloat("rollMid",   DEFAULT_ROLL_MID);
  ROLL_HIGH  = prefs.getFloat("rollHigh",  DEFAULT_ROLL_HIGH);
  prefs.end();
  Serial.printf("[CFG-ANG] Carregado: pitchHigh=%.1f rollLow=%.1f rollMid=%.1f rollHigh=%.1f\n",
                PITCH_HIGH, ROLL_LOW, ROLL_MID, ROLL_HIGH);
}

void saveAngleConfig() {
  prefs.begin("anglecfg", false);
  prefs.putFloat("pitchHigh", PITCH_HIGH);
  prefs.putFloat("rollLow",   ROLL_LOW);
  prefs.putFloat("rollMid",   ROLL_MID);
  prefs.putFloat("rollHigh",  ROLL_HIGH);
  prefs.end();
  Serial.println("[CFG-ANG] Salvo em NVS.");
}

// ── Bind por MAC (NVS) ────────────────────────────
// Guarda o endereco do carrinho vinculado, pra pular o scan por nome
// em conexoes seguintes e evitar pareamento cruzado com outros carrinhos
// que tambem comecem com "CARRINHO".
void loadBoundMac() {
  prefs.begin("blebind", true);
  boundMacStr = prefs.getString("mac", "");
  prefs.end();
  boundMacKnown = (boundMacStr.length() > 0);
  if (boundMacKnown) {
    Serial.printf("[BIND] MAC vinculado carregado: %s\n", boundMacStr.c_str());
  } else {
    Serial.println("[BIND] Nenhum MAC vinculado ainda.");
  }
}

void saveBoundMac(String mac) {
  prefs.begin("blebind", false);
  prefs.putString("mac", mac);
  prefs.end();
  Serial.printf("[BIND] MAC salvo: %s\n", mac.c_str());
}

void clearBoundMac() {
  prefs.begin("blebind", false);
  prefs.remove("mac");
  prefs.end();
  Serial.println("[BIND] Vinculo removido.");
}

void rebindRequest() {
  Serial.println("[BIND] Solicitando novo pareamento...");
  clearBoundMac();
  boundMacKnown = false;
  boundMacStr = "";
  bleConnected = false;
  currentMode = MODE_NORMAL;
  stopConfigWebServer();

  flashLED(0, 255, 255, 3, 150); // ciano = modo bind
  beep(4);

  if (pClient != nullptr) {
    pClient->disconnect();   // dispara onDisconnect -> needRescan (ja sem MAC vinculado)
  } else {
    startNameScan();
  }
}

// ── Web server de configuração (só ativo durante MODE_CONFIG) ──
void handleWebRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Config - Controle</title>";
  html += "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:16px;}"
          "input{width:80px;padding:4px;} label{display:block;margin-top:12px;}"
          ".live{background:#222;padding:10px;border-radius:6px;margin-bottom:14px;}"
          "button{margin-top:18px;padding:8px 16px;}</style></head><body>";
  html += "<h2>Ajuste de Angulos / Gatilhos</h2>";

  html += "<form action='/save' method='POST'>";
  html += "<label>Pitch limiar alto (ativa VF3/VB3):<input type='number' step='0.1' name='pitchHigh' value='" + String(PITCH_HIGH, 1) + "'></label>";
  html += "<label>Roll limiar baixo (zona morta / R0):<input type='number' step='0.1' name='rollLow' value='" + String(ROLL_LOW, 1) + "'></label>";
  html += "<label>Roll limiar medio:<input type='number' step='0.1' name='rollMid' value='" + String(ROLL_MID, 1) + "'></label>";
  html += "<label>Roll limiar alto:<input type='number' step='0.1' name='rollHigh' value='" + String(ROLL_HIGH, 1) + "'></label>";
  html += "<button type='submit'>Salvar</button>";
  html += "</form>";

  html += "<p style='margin-top:20px;color:#888;font-size:12px;'>Segure o botao trava por 5s no controle pra sair do modo config.</p>";
  html += "</body></html>";

  configServer.send(200, "text/html", html);
}

void handleWebSave() {
  if (configServer.hasArg("pitchHigh")) PITCH_HIGH = configServer.arg("pitchHigh").toFloat();
  if (configServer.hasArg("rollLow"))   ROLL_LOW   = configServer.arg("rollLow").toFloat();
  if (configServer.hasArg("rollMid"))   ROLL_MID   = configServer.arg("rollMid").toFloat();
  if (configServer.hasArg("rollHigh"))  ROLL_HIGH  = configServer.arg("rollHigh").toFloat();

  saveAngleConfig();

  configServer.sendHeader("Location", "/");
  configServer.send(303);
}

void startConfigWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONFIG_AP_SSID); // sem senha = AP aberto

  configServer.on("/", HTTP_GET, handleWebRoot);
  configServer.on("/save", HTTP_POST, handleWebSave);
  configServer.begin();
  webServerActive = true;

  Serial.print("[WEBCFG] AP ativo, SSID: ");
  Serial.println(CONFIG_AP_SSID);
  Serial.print("[WEBCFG] Acesse http://");
  Serial.println(WiFi.softAPIP());
}

void stopConfigWebServer() {
  if (!webServerActive) return;
  configServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  webServerActive = false;
  Serial.println("[WEBCFG] AP e servidor desligados.");
}

// ── MPU6050 ───────────────────────────────────────
void zeroAngles()
{
    readRawData();

    float ax = axRaw;
    float ay = ayRaw;
    float az = azRaw;

    pitchOffset = atan2(
      ay,
      sqrt(ax*ax + az*az)
    ) * 180.0f / PI;

    rollOffset = atan2(
      ax,
      sqrt(ay*ay + az*az)
    ) * 180.0f / PI;

    Serial.println("[MPU] Ângulos zerados");

    flashLED(100,180,0,1,250);
    beep(1);
}

void setupMPU()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    delay(100);

    uint8_t id = readRegister(REG_WHO_AM_I);

    Serial.print("[MPU] WHO_AM_I = 0x");
    Serial.println(id, HEX);

    if(id != 0x70 && id != 0x68)
    {
        Serial.println("[MPU] Sensor não reconhecido!");

        while(true)
        {
            flashLED(255,0,0,1,150);
            delay(150);
        }
    }

    // Wake-up
    writeRegister(REG_PWR_MGMT_1,0x00);

    delay(100);

    // Acelerômetro ±8g
    writeRegister(REG_ACCEL_CONFIG,0x10);

    // Giroscópio ±500°/s
    writeRegister(REG_GYRO_CONFIG,0x08);

    Serial.println("[MPU] Inicializado!");

    zeroAngles();
}

void readMPU()
{
    readRawData();

    float ax = axRaw;
    float ay = ayRaw;
    float az = azRaw;

    float rawPitch = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / PI;
    float rawRoll  = atan2(ax, sqrt(ay*ay + az*az)) * 180.0f / PI;

    roll = rawPitch - pitchOffset;
    pitch  = rawRoll  - rollOffset;
}

// ── Gatilho + IMU → comandos ──────────────────────
void calculateCommands() {
  bool fwd = (digitalRead(BTN_FRONT) == LOW);
  bool bwd = (digitalRead(BTN_BACK)  == LOW);

  if (fwd && !bwd) {
    if      (pitch >  PITCH_HIGH) velocityCmd = "VF1";
    else if (pitch < -PITCH_HIGH) velocityCmd = "VF3";
    else                          velocityCmd = "VF2";
  }
  else if (bwd && !fwd) {
    if      (pitch < -PITCH_HIGH) velocityCmd = "VB1";
    else if (pitch >  PITCH_HIGH) velocityCmd = "VB3";
    else                          velocityCmd = "VB2";
  }
  else {
    velocityCmd = "V0";
  }

  if      (roll >  ROLL_HIGH) rotationCmd = "R3";
  else if (roll >  ROLL_MID)  rotationCmd = "R2";
  else if (roll >  ROLL_LOW)  rotationCmd = "R1";
  else if (roll < -ROLL_HIGH) rotationCmd = "L3";
  else if (roll < -ROLL_MID)  rotationCmd = "L2";
  else if (roll < -ROLL_LOW)  rotationCmd = "L1";
  else                        rotationCmd = "0";
}

// ── BLE Client ────────────────────────────────────
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    String ack = String((char*)pData).substring(0, length);
    Serial.print("[ACK do Nano] ");
    Serial.println(ack);
    processAck(ack);
}

void processAck(String ack) {
  // Exemplo: se receber "ACK:HELLO", podemos reagir
  // Por enquanto, apenas exibe no serial.
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    bleConnected = true;
    Serial.print("[BLE] Conectado ao carrinho: ");
    Serial.println(connectedDeviceName);
    beep(2);
    // Envia um comando de saudação (opcional)
    sendCommandToNano("HELLO");
  }
  void onDisconnect(BLEClient* pclient) {
    bleConnected = false;
    Serial.println("[BLE] Desconectado.");
    pClient = nullptr;
    scanComplete = false;
    connectedDeviceName = "";
    needRescan = true;
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {

      // Filtro por NOME (prefixo). O modulo nao anuncia o UUID
      // do servico, entao filtrar por UUID no advertising nunca
      // funcionaria aqui.
      if (!advertisedDevice.haveName()) return;

      String devName = String(advertisedDevice.getName().c_str());
      if (!devName.startsWith(TARGET_NAME_PREFIX)) return;

      Serial.print("[BLE] Modulo compativel encontrado: ");
      Serial.print(devName);
      Serial.print(" | RSSI: ");
      Serial.println(advertisedDevice.getRSSI());

      BLEDevice::getScan()->stop();
      scanComplete = true;
      connectedDeviceName = devName;

      // NAO conectar aqui dentro. So guarda o device e sinaliza pro loop().
      if (pTargetDevice != nullptr) {
        delete pTargetDevice;
        pTargetDevice = nullptr;
      }
      pTargetDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
};

// Executa a conexao de fato. Chamada a partir do loop(), nunca de dentro
// de um callback do BLE stack.
bool connectToServer() {
  Serial.println("[BLE] Iniciando conexao...");

  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
  }

  bool connectOk = false;
  const int MAX_ATTEMPTS = 3;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS && !connectOk; attempt++) {
    Serial.printf("[BLE] Tentativa de conexao %d/%d...\n", attempt, MAX_ATTEMPTS);

    if (pTargetDevice != nullptr) {
      connectOk = pClient->connect(pTargetDevice);
    } else if (boundMacKnown) {
      BLEAddress targetAddr(boundMacStr.c_str());
      connectOk = pClient->connect(targetAddr, BLE_ADDR_PUBLIC);
    } else {
      Serial.println("[BLE] Nenhum alvo definido (nem device escaneado, nem MAC vinculado).");
      return false;
    }

    if (!connectOk && attempt < MAX_ATTEMPTS) {
      delay(500);
    }
  }

  if (!connectOk) {
    Serial.println("[BLE] Falha ao conectar apos varias tentativas.");
    return false;
  }

  Serial.println("[BLE] Conectado ao servidor!");

  // Primeira conexao por nome (ainda sem vinculo salvo): grava o bind agora.
  if (pTargetDevice != nullptr && !boundMacKnown) {
    boundMacStr = String(pTargetDevice->getAddress().toString().c_str());
    saveBoundMac(boundMacStr);
    boundMacKnown = true;
  }

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("[BLE] ERRO: Serviço 0xFFE0 não encontrado!");
    pClient->disconnect();
    return false;
  }

  pWriteCharacteristic = pRemoteService->getCharacteristic(charDataUUID);
  if (pWriteCharacteristic == nullptr) {
    Serial.println("[BLE] ERRO: Característica 0xFFE1 não encontrada!");
    pClient->disconnect();
    return false;
  }
  Serial.println("[BLE] Característica 0xFFE1 (escrita) encontrada.");

  if (pWriteCharacteristic->canNotify()) {
    pWriteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("[BLE] Inscrito para notificações em 0xFFE1.");
  } else {
    Serial.println("[BLE] AVISO: 0xFFE1 não respondeu como notify-capable nesta conexão.");
  }

  return true;
}

void startNameScan() {
  Serial.printf("[BLE] Procurando dispositivos com nome iniciando em \"%s\"\n", TARGET_NAME_PREFIX);
  BLEScan* pBLEScan = BLEDevice::getScan();
  if (!scanCallbackSet) {
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    scanCallbackSet = true;
  }
  pBLEScan->start(30, false);
}

void setupBLE_Client() {
  Serial.println("[BLE] Iniciando Cliente BLE...");
  BLEDevice::init("ESP32_Remote_Client");

  loadBoundMac();

  if (boundMacKnown) {
    Serial.println("[BLE] MAC vinculado presente -- conectando direto (sem scan).");
    doConnect = true;   // tratado no loop(), usa o caminho por boundMacStr
  } else {
    startNameScan();
  }
}

void sendCommandToNano(String cmd) {
  if (pWriteCharacteristic != nullptr && bleConnected) {
    Serial.print("[ESP -> Nano] ");
    Serial.println(cmd);
    // response=true: a caracteristica declara propriedade WRITE (com resposta),
    // nao WRITE_NR. Forcar isso evita falhas silenciosas de escrita.
    pWriteCharacteristic->writeValue((uint8_t*)cmd.c_str(), cmd.length(), true);
  } else {
    // Se não estiver conectado, tenta reconectar (já tratado pelo callback)
  }
}

// ── Botão trava (GPIO 10) ─────────────────────────
void handleLockButton() {
  bool current = digitalRead(BTN_LOCK);
  unsigned long now = millis();

  if (lastLockState == HIGH && current == LOW) {
    lockPressTime  = now;
    lockHoldFired  = false;
    lockHoldFired10 = false;
  }

  if (current == LOW && !lockHoldFired &&
      (now - lockPressTime >= 5000)) {
    lockHoldFired = true;
    waitingSecondClick = false;

    if (currentMode == MODE_CONFIG) {
      currentMode = MODE_NORMAL;
      stopConfigWebServer();
      flashLED(0, 255, 0, 2, 150);
      beep(3);
      Serial.println("[MODE] CONFIG → NORMAL");
    } else {
      currentMode = MODE_CONFIG;
      startConfigWebServer();
      flashLED(255, 180, 0, 3, 120);
      beep(2);
      Serial.println("[MODE] → CONFIG");
    }
  }

  // Segurar 10s (continuando pressionado alem dos 5s do CONFIG) = rebind
  if (current == LOW && !lockHoldFired10 &&
      (now - lockPressTime >= 10000)) {
    lockHoldFired10 = true;
    waitingSecondClick = false;
    Serial.println("[MODE] Hold 10s -> solicitando novo pareamento (bind)");
    rebindRequest();
  }

  if (lastLockState == LOW && current == HIGH) {
    unsigned long held = now - lockPressTime;

    if (!lockHoldFired && held >= 50) {
      if (waitingSecondClick && (now - firstClickTime < 400)) {
        waitingSecondClick = false;
        if (currentMode == MODE_CONFIG) {
          currentMode = MODE_NORMAL;
          stopConfigWebServer();
          flashLED(0, 255, 0, 2, 150);
          beep(3);
          Serial.println("[MODE] CONFIG → NORMAL (duplo clique)");
        } else {
          if (currentMode == MODE_LOCK) {
            currentMode = MODE_NORMAL;
            zeroAngles();
            beep(3);
            Serial.println("[MODE] UNLOCK");
          } else {
            currentMode = MODE_LOCK;
            velocityCmd = "VF0";
            rotationCmd = "R0";
            beep(2);
            Serial.println("[MODE] LOCK");
          }
        }
      }
      else {
        waitingSecondClick = true;
        firstClickTime = now;
      }
    }
  }

  if (waitingSecondClick && (now - firstClickTime > 400)) {
    waitingSecondClick = false;
    if (currentMode == MODE_NORMAL) {
      zeroAngles();
      Serial.println("[BTN] Clique simples → CAL");
    }
  }

  lastLockState = current;
}

// ══════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32-S3 Remote v2.3 — Cliente BLE + Web Config");

  pinMode(BTN_FRONT, INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);
  pinMode(BTN_LOCK,  INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  setupLED();
  loadConfig();
  loadAngleConfig();
  setupMPU();
  setupBLE_Client();

  flashLED(255, 0,   0,   1, 200);
  flashLED(0,   255, 0,   1, 200);
  flashLED(0,   0,   255, 1, 200);
  beep(2);

  Serial.println("[BOOT] Sistema pronto");
}

// ══════════════════════════════════════════════════
void loop() {
  // Conexao de fato acontece aqui, nunca dentro do callback do scan.
  if (doConnect) {
    doConnect = false;
    if (!connectToServer()) {
      scanComplete = false;
      connectedDeviceName = "";
      needRescan = true;
    }
  }

  if (needRescan) {
    needRescan = false;
    Serial.println("[BLE] Tentando reconectar em 2s...");
    delay(2000);
    if (boundMacKnown) {
      doConnect = true;   // reconecta direto por MAC, sem precisar escanear de novo
    } else {
      startNameScan();
    }
  }

  handleLockButton();

  // Le o MPU sempre, mesmo em CONFIG -- necessario pra tela web mostrar
  // pitch/roll ao vivo durante a calibracao.
  readMPU();

  if (currentMode == MODE_NORMAL) {
    calculateCommands();
  } else if (currentMode == MODE_LOCK) {
    velocityCmd = "VF0";
    rotationCmd = "R0";
  }
  // MODE_CONFIG: nao calcula comandos, so mantem pitch/roll atualizados

  updateLED();

  if (currentMode == MODE_CONFIG && webServerActive) {
    configServer.handleClient();
  }

  // Envia comandos para o Nano a cada 10ms (se conectado e em modo normal)
  if (currentMode == MODE_NORMAL && bleConnected && (millis() - lastSend >= SEND_INTERVAL)) {
    String cmd = velocityCmd + "," + rotationCmd;
    sendCommandToNano(cmd);
    lastSend = millis();
  }
}
