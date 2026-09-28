/*
  RoverLab - firmware V3
  Arduino Nano + TB6612FNG + HM-10

  Protocolo V3 ASCII enquadrado:
    @D,V=2,R=0,S=1*CRC;
    @C,UNLOCK*CRC;
    @C,PROFILE,35,80,135*CRC;
    @C,GET*CRC;
    @STOP*CRC;

  CRC8 e soma modulo 256 dos bytes do payload, sem '@', '*' ou ';'.
  O CRC e transmitido em dois digitos hexadecimais.

  Compatibilidade opcional com o protocolo antigo:
    VF2,R1 / VB1,L2 / V0,0
  Ative LEGACY_COMPAT somente durante a migracao.

  Pinagem:
    Motor esquerdo: AIN1=A3, AIN2=A4, PWMA=D3
    Motor direito:  BIN1=A1, BIN2=A2, PWMB=D5
    BLE STATE: D7
    HM-10: Serial D0/D1, 38400 baud
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <string.h>
#include "Config.h"
#include "Leds.h"

const uint8_t AIN1 = MOTOR_LEFT_IN1, AIN2 = MOTOR_LEFT_IN2, PWMA = MOTOR_LEFT_PWM;
const uint8_t BIN1 = MOTOR_RIGHT_IN1, BIN2 = MOTOR_RIGHT_IN2, PWMB = MOTOR_RIGHT_PWM;
const uint8_t BLE_STATE = BLE_STATE_PIN;

const bool LEGACY_COMPAT = true;
const uint32_t FAILSAFE_MS = MOTOR_FAILSAFE_MS;
const uint32_t UNLOCK_MS = CONFIG_UNLOCK_MS;
const uint8_t FRAME_MAX = 180;
const uint8_t PWM_DEFAULT[4] = {0, DEFAULT_PWM_MIN, DEFAULT_PWM_MED, DEFAULT_PWM_MAX};

// Programa: cada instrução ocupa 3 bytes: opcode, parâmetro e duração em 100 ms.
// Os slots ficam depois dos slots do perfil no EEPROM de 1 KB do Nano.
const uint8_t PROGRAM_MAGIC = 0xB6;
const uint8_t PROGRAM_VERSION = 1;
const uint16_t PROGRAM_SLOT_A = 32;
const uint16_t PROGRAM_SLOT_B = 256;
const uint16_t PROGRAM_HEADER_SIZE = 10;
const uint16_t MAX_PROGRAM_BYTES = 160;
const uint8_t OP_FORWARD = 0x01;
const uint8_t OP_BACKWARD = 0x02;
const uint8_t OP_LEFT = 0x03;
const uint8_t OP_RIGHT = 0x04;
const uint8_t OP_STOP = 0x05;
const uint8_t OP_WAIT = 0x06;
const uint8_t OP_END = 0xFF;

// Dois slots permitem recuperar o perfil anterior se faltar energia durante a escrita.
const uint8_t PROFILE_MAGIC = 0xA7;
const uint16_t SLOT_A = 0;
const uint16_t SLOT_B = 16;
struct ProfileRecord {
  uint8_t magic;
  uint8_t version;
  uint8_t sequence;
  uint8_t minPwm;
  uint8_t medPwm;
  uint8_t maxPwm;
  uint8_t crc;
};

uint8_t pwmLevel[4] = {0, DEFAULT_PWM_MIN, DEFAULT_PWM_MED, DEFAULT_PWM_MAX};
uint8_t currentLevel = 3;
uint8_t activeSequence = 0;
uint16_t activeSlot = SLOT_A;
uint32_t lastValidCommand = 0;
uint32_t unlockUntil = 0;

uint8_t uploadBuffer[MAX_PROGRAM_BYTES];
uint16_t uploadLength = 0;
uint16_t uploadExpectedLength = 0;
uint16_t uploadExpectedCrc = 0;
bool uploadOpen = false;
uint16_t activeProgramLength = 0;
uint16_t activeProgramCrc = 0;
uint8_t activeProgramSequence = 0;
uint16_t activeProgramSlot = PROGRAM_SLOT_A;
bool programRunning = false;
uint16_t programPc = 0;
uint32_t programStepUntil = 0;

char frame[FRAME_MAX + 1];
uint8_t frameLength = 0;
bool inFrame = false;

void reply(const char *payload);
void stopMotors();
void applyDrive(int8_t velocity, int8_t rotation);

uint8_t crc8(const char *text) {
  uint16_t sum = 0;
  while (*text) sum += (uint8_t)*text++;
  return (uint8_t)(sum & 0xFF);
}

uint16_t crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= (uint16_t)*data++ << 8;
    for (uint8_t bit = 0; bit < 8; bit++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

uint16_t programHeaderCrc(uint16_t length, uint16_t programCrc, uint8_t sequence) {
  uint8_t header[5] = {PROGRAM_MAGIC, PROGRAM_VERSION, sequence, (uint8_t)(length >> 8), (uint8_t)length};
  header[3] = (uint8_t)(length >> 8);
  header[4] = (uint8_t)length;
  uint16_t crc = crc16(header, sizeof(header));
  uint8_t crcBytes[2] = {(uint8_t)(programCrc >> 8), (uint8_t)programCrc};
  return crc16(crcBytes, 2) ^ crc;
}

bool validProgramHeader(uint16_t slot, uint16_t *length, uint16_t *programCrc, uint8_t *sequence) {
  uint8_t magic = EEPROM.read(slot);
  uint8_t version = EEPROM.read(slot + 1);
  if (magic != PROGRAM_MAGIC || version != PROGRAM_VERSION) return false;
  *sequence = EEPROM.read(slot + 2);
  *length = ((uint16_t)EEPROM.read(slot + 3) << 8) | EEPROM.read(slot + 4);
  *programCrc = ((uint16_t)EEPROM.read(slot + 5) << 8) | EEPROM.read(slot + 6);
  uint16_t storedHeaderCrc = ((uint16_t)EEPROM.read(slot + 7) << 8) | EEPROM.read(slot + 8);
  if (*length == 0 || *length > MAX_PROGRAM_BYTES) return false;
  uint8_t data[MAX_PROGRAM_BYTES];
  for (uint16_t i = 0; i < *length; i++) data[i] = EEPROM.read(slot + PROGRAM_HEADER_SIZE + i);
  return crc16(data, *length) == *programCrc && programHeaderCrc(*length, *programCrc, *sequence) == storedHeaderCrc;
}

bool newerSequence(uint8_t candidate, uint8_t current) {
  return (uint8_t)(candidate - current) < 128 && candidate != current;
}

void loadProgram() {
  uint16_t lengthA = 0, lengthB = 0, crcA = 0, crcB = 0;
  uint8_t sequenceA = 0, sequenceB = 0;
  bool validA = validProgramHeader(PROGRAM_SLOT_A, &lengthA, &crcA, &sequenceA);
  bool validB = validProgramHeader(PROGRAM_SLOT_B, &lengthB, &crcB, &sequenceB);
  if (!validA && !validB) { activeProgramLength = 0; return; }
  bool chooseB = validB && (!validA || newerSequence(sequenceB, sequenceA));
  activeProgramSlot = chooseB ? PROGRAM_SLOT_B : PROGRAM_SLOT_A;
  activeProgramLength = chooseB ? lengthB : lengthA;
  activeProgramCrc = chooseB ? crcB : crcA;
  activeProgramSequence = chooseB ? sequenceB : sequenceA;
}

void writeProgramSlot(uint16_t slot, const uint8_t *data, uint16_t length, uint16_t programCrc, uint8_t sequence) {
  // Dados são escritos antes do cabeçalho. O magic só é marcado por último.
  EEPROM.update(slot, 0);
  for (uint16_t i = 0; i < length; i++) EEPROM.update(slot + PROGRAM_HEADER_SIZE + i, data[i]);
  EEPROM.update(slot + 1, PROGRAM_VERSION);
  EEPROM.update(slot + 2, sequence);
  EEPROM.update(slot + 3, length >> 8);
  EEPROM.update(slot + 4, length & 0xFF);
  EEPROM.update(slot + 5, programCrc >> 8);
  EEPROM.update(slot + 6, programCrc & 0xFF);
  uint16_t headerCrc = programHeaderCrc(length, programCrc, sequence);
  EEPROM.update(slot + 7, headerCrc >> 8);
  EEPROM.update(slot + 8, headerCrc & 0xFF);
  EEPROM.update(slot + 9, OP_END);
  EEPROM.update(slot, PROGRAM_MAGIC);
}

bool validProgramInstruction(uint16_t pc) {
  if (pc + 2 >= uploadLength) return false;
  uint8_t opcode = uploadBuffer[pc];
  uint8_t level = uploadBuffer[pc + 1];
  if (opcode < OP_FORWARD || opcode > OP_WAIT) return opcode == OP_END;
  if (opcode == OP_STOP) return level == 0;
  return opcode == OP_WAIT ? level == 0 : level >= 1 && level <= 3;
}

void stopProgram() {
  programRunning = false;
  stopMotors();
}

void startProgram() {
  if (!activeProgramLength) { reply("ERR,NO_PROGRAM"); return; }
  programRunning = true;
  programPc = 0;
  programStepUntil = millis();
  reply("OK,P,START");
}

void tickProgram() {
  if (!programRunning || (int32_t)(millis() - programStepUntil) < 0) return;
  if (programPc + 2 >= activeProgramLength) { stopProgram(); reply("OK,P,DONE"); return; }
  uint8_t opcode = EEPROM.read(activeProgramSlot + PROGRAM_HEADER_SIZE + programPc);
  uint8_t level = EEPROM.read(activeProgramSlot + PROGRAM_HEADER_SIZE + programPc + 1);
  uint8_t units = EEPROM.read(activeProgramSlot + PROGRAM_HEADER_SIZE + programPc + 2);
  programPc += 3;
  if (opcode == OP_END) { stopProgram(); reply("OK,P,DONE"); return; }
  if (opcode == OP_FORWARD) applyDrive(level, 0);
  else if (opcode == OP_BACKWARD) applyDrive(-level, 0);
  else if (opcode == OP_LEFT) applyDrive(0, -level);
  else if (opcode == OP_RIGHT) applyDrive(0, level);
  else if (opcode == OP_STOP || opcode == OP_WAIT) stopMotors();
  else { stopProgram(); reply("ERR,P_BAD_OPCODE"); return; }
  programStepUntil = millis() + (uint32_t)max((uint8_t)1, units) * 100UL;
}

int hexValue(char c);

bool decodeHexByte(char high, char low, uint8_t *value) {
  int h = hexValue(high), l = hexValue(low);
  if (h < 0 || l < 0) return false;
  *value = (uint8_t)((h << 4) | l);
  return true;
}

void processProgramCommand(char *payload) {
  if (strcmp(payload, "P,INFO") == 0) {
    char response[64];
    snprintf(response, sizeof(response), "OK,P,INFO,%u,%04X", activeProgramLength, activeProgramCrc);
    reply(response);
    return;
  }
  if (strcmp(payload, "P,BEGIN") == 0) {
    uploadOpen = true; uploadLength = 0; uploadExpectedLength = 0; uploadExpectedCrc = 0;
    reply("OK,P,BEGIN");
    return;
  }
  if (strncmp(payload, "P,BEGIN,", 8) == 0) {
    unsigned int length = 0, expectedCrc = 0;
    if (sscanf(payload + 8, "%u,%x", &length, &expectedCrc) != 2 || length == 0 || length > MAX_PROGRAM_BYTES) { reply("ERR,P_BEGIN"); return; }
    uploadOpen = true; uploadLength = 0; uploadExpectedLength = length; uploadExpectedCrc = expectedCrc;
    reply("OK,P,BEGIN");
    return;
  }
  if (strncmp(payload, "P,DATA,", 7) == 0) {
    if (!uploadOpen) { reply("ERR,P_NO_BEGIN"); return; }
    unsigned int offset = 0;
    char hex[55] = {0};
    if (sscanf(payload + 7, "%u,%54s", &offset, hex) != 2) { reply("ERR,P_DATA"); return; }
    uint16_t byteCount = strlen(hex) / 2;
    if (strlen(hex) % 2 || offset + byteCount > MAX_PROGRAM_BYTES) { reply("ERR,P_RANGE"); return; }
    for (uint16_t i = 0; i < byteCount; i++) if (!decodeHexByte(hex[i * 2], hex[i * 2 + 1], &uploadBuffer[offset + i])) { reply("ERR,P_HEX"); return; }
    if (offset + byteCount > uploadLength) uploadLength = offset + byteCount;
    reply("OK,P,DATA");
    return;
  }
  if (strcmp(payload, "P,COMMIT") == 0) {
    bool instructionsOk = uploadLength % 3 == 0;
    for (uint16_t pc = 0; instructionsOk && pc < uploadLength; pc += 3) instructionsOk = validProgramInstruction(pc);
    if (!uploadOpen || !instructionsOk || uploadLength != uploadExpectedLength || crc16(uploadBuffer, uploadLength) != uploadExpectedCrc) { uploadOpen = false; reply("ERR,P_CRC"); return; }
    uint16_t previousSlot = activeProgramSlot;
    uint16_t nextSlot = previousSlot == PROGRAM_SLOT_A ? PROGRAM_SLOT_B : PROGRAM_SLOT_A;
    uint8_t nextSequence = activeProgramSequence + 1;
    writeProgramSlot(nextSlot, uploadBuffer, uploadLength, uploadExpectedCrc, nextSequence);
    loadProgram(); uploadOpen = false; reply("OK,P,COMMIT");
    return;
  }
  if (strcmp(payload, "P,START") == 0) { startProgram(); return; }
  if (strcmp(payload, "P,STOP") == 0) { stopProgram(); reply("OK,P,STOP"); return; }
  reply("ERR,P_UNKNOWN");
}

char hexDigit(uint8_t n) {
  return n < 10 ? '0' + n : 'A' + n - 10;
}

void printHex8(uint8_t value) {
  Serial.write(hexDigit(value >> 4));
  Serial.write(hexDigit(value & 0x0F));
}

void reply(const char *payload) {
  Serial.write('@');
  Serial.print(payload);
  Serial.write('*');
  printHex8(crc8(payload));
  Serial.write(';');
}

void motor(uint8_t in1, uint8_t in2, uint8_t pwm, int value) {
  value = constrain(value, -255, 255);
  if (value > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwm, value);
  } else if (value < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwm, -value);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwm, 0);
  }
}

void stopMotors() {
  motor(AIN1, AIN2, PWMA, 0);
  motor(BIN1, BIN2, PWMB, 0);
}

int levelPwm(int8_t level) {
  if (level < -3 || level > 3) return 0;
  return (level < 0 ? -1 : 1) * pwmLevel[abs(level)];
}

bool parseSignedLevel(const char *text, int8_t *out) {
  if (!text || !*text) return false;
  bool negative = false;
  if (*text == '-') { negative = true; text++; }
  if (*text < '0' || *text > '3' || text[1] != '\0') return false;
  int8_t value = (int8_t)(*text - '0');
  *out = negative ? -value : value;
  return true;
}

void applyDrive(int8_t velocity, int8_t rotation) {
  currentLevel = max(abs(velocity), abs(rotation));
  int left = constrain(levelPwm(velocity) + levelPwm(rotation), -255, 255);
  int right = constrain(levelPwm(velocity) - levelPwm(rotation), -255, 255);
  motor(AIN1, AIN2, PWMA, left);
  motor(BIN1, BIN2, PWMB, right);
  lastValidCommand = millis();
}

uint8_t profileCrc(const ProfileRecord &record) {
  const uint8_t *bytes = (const uint8_t *)&record;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < sizeof(ProfileRecord) - 1; i++) sum += bytes[i];
  return (uint8_t)(sum & 0xFF);
}

bool validRecord(const ProfileRecord &record) {
  return record.magic == PROFILE_MAGIC && record.version == 1 && record.crc == profileCrc(record) && record.minPwm <= record.medPwm && record.medPwm <= record.maxPwm;
}

void saveProfile() {
  ProfileRecord record = {PROFILE_MAGIC, 1, (uint8_t)(activeSequence + 1), pwmLevel[1], pwmLevel[2], pwmLevel[3], 0};
  record.crc = profileCrc(record);
  uint16_t nextSlot = activeSlot == SLOT_A ? SLOT_B : SLOT_A;
  EEPROM.put(nextSlot, record);
  activeSlot = nextSlot;
  activeSequence = record.sequence;
}

void loadProfile() {
  ProfileRecord a, b;
  EEPROM.get(SLOT_A, a);
  EEPROM.get(SLOT_B, b);
  bool validA = validRecord(a), validB = validRecord(b);
  if (!validA && !validB) {
    pwmLevel[1] = PWM_DEFAULT[1]; pwmLevel[2] = PWM_DEFAULT[2]; pwmLevel[3] = PWM_DEFAULT[3];
    activeSlot = SLOT_A; activeSequence = 0; saveProfile();
    return;
  }
  ProfileRecord chosen = validB && (!validA || (uint8_t)(b.sequence - a.sequence) < 128) ? b : a;
  pwmLevel[1] = chosen.minPwm; pwmLevel[2] = chosen.medPwm; pwmLevel[3] = chosen.maxPwm;
  activeSlot = (&chosen == &b) ? SLOT_B : SLOT_A;
  // O endereco e definido pelo registro valido escolhido, sem usar ponteiro apos a copia.
  if (validB && (!validA || (uint8_t)(b.sequence - a.sequence) < 128)) activeSlot = SLOT_B; else activeSlot = SLOT_A;
  activeSequence = chosen.sequence;
}

bool fieldValue(const char *payload, const char *key, int8_t *value) {
  const char *start = strstr(payload, key);
  if (!start) return false;
  start += strlen(key);
  char buffer[5] = {0};
  uint8_t i = 0;
  while (*start && *start != ',' && i < sizeof(buffer) - 1) buffer[i++] = *start++;
  return i > 0 && parseSignedLevel(buffer, value);
}

void sendProfile() {
  char response[64];
  snprintf(response, sizeof(response), "OK,PROFILE,%u,%u,%u", pwmLevel[1], pwmLevel[2], pwmLevel[3]);
  reply(response);
}

void processPayload(char *payload) {
  if (payload[0] == 'P' && (payload[1] == ',' || payload[1] == '\0')) { processProgramCommand(payload); return; }

  if (strncmp(payload, "D,", 2) == 0) {
    int8_t velocity, rotation;
    if (!fieldValue(payload, "V=", &velocity) || !fieldValue(payload, "R=", &rotation)) { reply("ERR,BAD_DRIVE"); return; }
    applyDrive(velocity, rotation);
    reply("OK,D");
    return;
  }

  if (strcmp(payload, "STOP") == 0) {
    stopProgram(); lastValidCommand = millis(); reply("OK,STOP"); return;
  }

  if (strcmp(payload, "C,UNLOCK") == 0) {
    unlockUntil = millis() + UNLOCK_MS; reply("OK,UNLOCK"); return;
  }

  if (strcmp(payload, "C,GET") == 0) { sendProfile(); return; }

  if (strncmp(payload, "C,PROFILE,", 10) == 0) {
    if ((int32_t)(millis() - unlockUntil) >= 0) { reply("ERR,LOCKED"); return; }
    int minPwm, medPwm, maxPwm;
    if (sscanf(payload + 10, "%d,%d,%d", &minPwm, &medPwm, &maxPwm) != 3 || minPwm < 0 || medPwm < minPwm || maxPwm < medPwm || maxPwm > 255) { reply("ERR,BAD_PROFILE"); return; }
    pwmLevel[1] = minPwm; pwmLevel[2] = medPwm; pwmLevel[3] = maxPwm;
    saveProfile(); unlockUntil = 0; sendProfile(); return;
  }

  reply("ERR,UNKNOWN");
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void processFrame(char *raw) {
  char *star = strrchr(raw, '*');
  if (!star || star == raw || strlen(star + 1) != 2) { reply("ERR,BAD_FRAME"); return; }
  int hi = hexValue(star[1]), lo = hexValue(star[2]);
  *star = '\0';
  if (hi < 0 || lo < 0 || crc8(raw) != (uint8_t)((hi << 4) | lo)) { reply("ERR,BAD_CRC"); return; }
  processPayload(raw);
}

void processLegacy(char c) {
  // Compatibilidade minima: o firmware V3 continua entendendo V0,0 e VF2,R1.
  static char buffer[16]; static uint8_t length = 0;
  if (c == 'V') { length = 0; buffer[length++] = c; return; }
  if (!length) return;
  if (c == 'V' || length >= sizeof(buffer) - 1) { length = 0; return; }
  buffer[length++] = c; buffer[length] = '\0';
  if (c == ',' || (length > 3 && c != ',')) return;
  char *comma = strchr(buffer, ',');
  if (!comma) return;
  if (strlen(comma + 1) < 1) return;
  if (strlen(comma + 1) >= 1 && (c == '0' || length >= 5)) {
    int8_t v = 0, r = 0;
    char vText[5] = {0}, rText[4] = {0};
    strncpy(vText, buffer, comma - buffer); strncpy(rText, comma + 1, sizeof(rText) - 1);
    if (strcmp(vText, "V0") == 0) v = 0;
    else if (vText[0] == 'V' && (vText[1] == 'F' || vText[1] == 'B')) { v = vText[2] - '0'; if (vText[1] == 'B') v = -v; }
    else return;
    if (rText[0] == '0') r = 0; else if (rText[0] == 'R' || rText[0] == 'L') { r = rText[1] - '0'; if (rText[0] == 'L') r = -r; } else return;
    applyDrive(v, r); length = 0;
  }
}

void processByte(char c) {
  static char legacyChar = 0;
  if (c == '@') { inFrame = true; frameLength = 0; return; }
  if (inFrame) {
    if (c == ';') { frame[frameLength] = '\0'; processFrame(frame); inFrame = false; frameLength = 0; return; }
    if (frameLength >= FRAME_MAX) { inFrame = false; frameLength = 0; reply("ERR,TOO_LONG"); return; }
    frame[frameLength++] = c;
    return;
  }
  if (LEGACY_COMPAT) processLegacy(c);
}

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(BLE_STATE, INPUT);
  RoverLeds::begin();
  stopMotors(); loadProfile(); loadProgram(); Serial.begin(BLE_BAUD); lastValidCommand = millis();
  reply("READY,V3");
}

void loop() {
  while (Serial.available()) processByte((char)Serial.read());
  tickProgram();
  // Durante um programa, o executor mantém o movimento válido.
  // O watchdog de 300 ms protege somente o modo manual.
  if (programRunning) lastValidCommand = millis();
  else if ((uint32_t)(millis() - lastValidCommand) > FAILSAFE_MS) stopMotors();
  RoverLeds::update(digitalRead(BLE_STATE) == BLE_CONNECTED_LEVEL, (int32_t)(millis() - unlockUntil) < 0, programRunning, uploadOpen, false, currentLevel);
}
