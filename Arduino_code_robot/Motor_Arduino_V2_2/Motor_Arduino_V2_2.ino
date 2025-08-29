// ============================================================
// Author      : Baptiste Poncet
// Date        : 04/08/2025
// File        : Motor_Arduino_V2_1.ino
// Version     : 2.2
// Description : Code linking LoRa communication (UART) and motor (CANopen protocole)
//               with encoders informations/ Autonomous reading of encoders using TPDO
//               timely display, both direction, ekf friendly, ground truth
// ============================================================

#include <SPI.h>
#include <mcp_can.h>

#define CS_PIN 10
MCP_CAN CAN(CS_PIN);

String inputBuffer = "";

bool serialBusy = false;
unsigned long lastSerialWrite = 0;
const unsigned long SERIAL_DELAY = 10;

float T_init;
float T_ground_init;
float T_ground;
bool first;
bool commandePrioritaireRecue = false;
unsigned long commandeTimestamp = 0;
const unsigned long commandeTimeout = 100;

const float TICKS_PER_REV = 65536.0;
const float WHEEL_DIAMETER = 0.13;    // m
const float WHEEL_CIRCUM = PI * WHEEL_DIAMETER;
const float WHEEL_BASE = 0.30;
const float TRANSLATION_SPEED = 0.035; // m/s
const float ROTATION_SPEED = 0.2; //rad/s


float cumulative_distance = 0.0;

unsigned long last_encoder_check = 0;
unsigned long last_send_time = 0;
float accumulated_delta_L = 0.0;
float accumulated_delta_R = 0.0;
unsigned long lastNMT = 0;
int x = 1;
int stage = 0;

// ==================== STRUCTURES ======================
struct EncoderData {
  uint32_t can_id;
  int32_t last_position;
  bool first_read;
  float delta_meters;
  bool indic;

  EncoderData(uint32_t id) : can_id(id), last_position(0), first_read(true), delta_meters(0.0), indic(false) {}
};

EncoderData encoders[] = {
  EncoderData(0x381),
  EncoderData(0x382)
};

// ==================== SETUP ===========================
void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial);
  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN INIT FAILED");
    while (1);
  }

  CAN.init_Mask(0, 0, 0x7FF);       // Masque complet
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(0, 0, 0x381);       // Filtre sur 0x381
  CAN.init_Filt(1, 0, 0x382);       // Filtre sur 0x382
  CAN.init_Filt(1, 1, 0x701);  // Heartbeat node 1
  CAN.init_Filt(1, 2, 0x702);  // Heartbeat node 2
  CAN.setMode(MCP_NORMAL);

  //Motor modes to Pre-Operationnal
  byte preOpNode1[] = {0x80, 0x01};
  byte preOpNode2[] = {0x80, 0x02};
  CAN.sendMsgBuf(0x000, 0, 2, preOpNode1); delay(100);
  CAN.sendMsgBuf(0x000, 0, 2, preOpNode2); delay(100);

  Motor_init(0x01);
  Motor_init(0x02);
  delay(200);

  //Motor Modes to Operational
  byte opNode1[] = {0x01, 0x01};
  byte opNode2[] = {0x01, 0x02};
  CAN.sendMsgBuf(0x000, 0, 2, opNode1); delay(100);
  CAN.sendMsgBuf(0x000, 0, 2, opNode2); delay(100);

  byte dummyByte[1] = {0x00};
  CAN.sendMsgBuf(0x080, 0, 1, dummyByte);
  delay(100);

  Serial.println("Setup complete");
  T_init = millis();
  cumulative_distance = 0.0;
}

// ==================== LOOP =============================
void loop() {
  checkSerialInput();
  readCANMessages();  // Toujours lire les trames CAN

  if (commandePrioritaireRecue && millis() - commandeTimestamp < commandeTimeout) {
    return; // On saute les calculs mais on lit toujours le CAN
  }

  if (millis() - lastNMT > 3000) {
    byte opNode1[] = {0x01, 0x01};
    byte opNode2[] = {0x01, 0x02};
    CAN.sendMsgBuf(0x000, 0, 2, opNode1);
    CAN.sendMsgBuf(0x000, 0, 2, opNode2);
    lastNMT = millis();
  }


  updateOdometry(); // Calculs basés sur les données reçues
}

// ==================== FONCTIONS PRINCIPALES ====================

void readCANMessages() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);


    if (rxId == 0x701 || rxId == 0x702) {
      byte state = rxBuf[0];
      byte nodeId = rxId - 0x700;

      if (state != 0x05) {
        Serial.print("[WARN] Node ");
        Serial.print(nodeId);
        Serial.print(" not operational, state: 0x");
        Serial.println(state, HEX);

        // Réenvoi d’une commande NMT Start
        byte startCmd[] = {0x01, nodeId};
        CAN.sendMsgBuf(0x000, 0, 2, startCmd);
      }
    }

    for (int i = 0; i < 2; i++) {
      EncoderData &enc = encoders[i];

      if (rxId == enc.can_id && len >= 4) {
        int32_t pos_raw = (int32_t)((rxBuf[3] << 24) | (rxBuf[2] << 16) | (rxBuf[1] << 8) | rxBuf[0]);

        if (enc.first_read) {
          enc.last_position = pos_raw;
          enc.first_read = false;
        } else {
          int32_t delta_ticks = pos_raw - enc.last_position;
          enc.delta_meters = (delta_ticks / TICKS_PER_REV) * WHEEL_CIRCUM;
          enc.last_position = pos_raw;
          enc.indic = true;
        }
      }
    }
  }
}

void updateOdometry() {
  static unsigned long last_encoder_check = 0;
  unsigned long now = millis();

  // On déclenche si on a au moins une nouvelle roue OU si 50ms sont passées
  bool anyNew = encoders[0].indic || encoders[1].indic;
  if (anyNew || (now - last_encoder_check >= 50)) {
    last_encoder_check = now;

    // N'utiliser que les deltas "neufs"
    float delta_L = encoders[0].indic ? -encoders[0].delta_meters : 0.0f;
    float delta_R = encoders[1].indic ?  encoders[1].delta_meters : 0.0f;

    // Accumulation seulement si on a quelque chose de nouveau
    if (fabs(delta_L) > 1e-4f || fabs(delta_R) > 1e-4f) {
      accumulated_delta_L += delta_L;
      accumulated_delta_R += delta_R;
    }

    encoders[0].delta_meters = 0.0f;
    encoders[1].delta_meters = 0.0f;
    encoders[0].indic = false;
    encoders[1].indic = false;
  }

  // Envoi périodique (inchangé)
  static unsigned long last_send_time = 0;
  if (now - last_send_time >= 1500) {
    last_send_time = now;

    float delta_d    = (accumulated_delta_L + accumulated_delta_R) / 2.0f;
    float delta_theta = (accumulated_delta_R - accumulated_delta_L) / WHEEL_BASE;
    cumulative_distance += fabs(delta_d);

    if (cumulative_distance >= 0.30f) {
      Serial.println("[DIST]");
      cumulative_distance = 0.0f;
    }

    Serial.print("EKF ");
    Serial.print(delta_d, 6);
    Serial.print(" ");
    Serial.println(delta_theta, 6);

    accumulated_delta_L = 0.0f;
    accumulated_delta_R = 0.0f;
  }
}

// ==================== COMMANDES CAN ============================
void Motor_init(byte id) {
  uint16_t cob_id = 0x600 + id;
  uint32_t tpdo_id = 0x180 + id;

  send(cob_id, "2B 40 60 00 80 00 00 00"); delay(50);
  send(cob_id, "2B 40 60 00 06 00 00 00"); delay(50);
  send(cob_id, "2F 60 60 00 03 00 00 00"); delay(50);
  send(cob_id, "2B 40 60 00 07 00 00 00"); delay(50);

  sendSDO(cob_id, 0x1800, 0x01, 0x80000180 | id, 4); delay(50);
  sendSDO(cob_id, 0x1A00, 0x00, 0x00, 1); delay(50);
  sendSDO(cob_id, 0x1A00, 0x01, 0x60640020, 4); delay(50);
  sendSDO(cob_id, 0x1A00, 0x00, 0x01, 1); delay(50);
  sendSDO(cob_id, 0x1800, 0x02, 0xFE, 1); delay(50);
  sendSDO(cob_id, 0x1800, 0x05, 100, 2); delay(50);
  sendSDO(cob_id, 0x1800, 0x01, 0x180 + id, 4); delay(50);

  send(cob_id, "2B 40 60 00 0F 00 00 00"); delay(50);
}

void send(uint16_t id, const char *hexstr) {
  byte data[8] = {0};
  int index = 0;
  const char *p = hexstr;
  while (*p && index < 8) {
    while (*p == ' ') p++;
    if (!*p) break;
    byte b = strtoul(p, NULL, 16);
    data[index++] = b;
    while (*p && *p != ' ') p++;
  }
  CAN.sendMsgBuf(id, 0, 8, data);
}

void sendSDO(uint16_t cob_id, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
  byte data[8];
  switch (size) {
    case 1: data[0] = 0x2F; break;
    case 2: data[0] = 0x2B; break;
    case 4: data[0] = 0x23; break;
    default: return;
  }

  data[1] = index & 0xFF;
  data[2] = index >> 8;
  data[3] = subindex;
  data[4] = value & 0xFF;
  data[5] = (value >> 8) & 0xFF;
  data[6] = (value >> 16) & 0xFF;
  data[7] = (value >> 24) & 0xFF;

  CAN.sendMsgBuf(cob_id, 0, 8, data);
}

// ==================== COMMANDES SERIE ========================
void checkSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        inputBuffer.trim();
        if (inputBuffer.startsWith("SPEED")) {
          commandePrioritaireRecue = true;
          commandeTimestamp = millis();
        }
        processSerialCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void processSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("SPEED")) {
    int sep1 = cmd.indexOf(' ');
    int sep2 = cmd.indexOf(' ', sep1 + 1);
    if (sep1 == -1 || sep2 == -1) {
      Serial.println("[ERROR] Wrong Format");
      return;
    }

    int speed1 = cmd.substring(sep1 + 1, sep2).toInt();
    int speed2 = cmd.substring(sep2 + 1).toInt();

    String speed1Hex = intToHexString(speed1);
    String speed2Hex = intToHexString(speed2);

    Serial.println("[SPEED] Motor1: " + String(speed1) + " Motor2: " + String(speed2));
    send(0x601, ("23 FF 60 00 " + speed1Hex).c_str());
    send(0x602, ("23 FF 60 00 " + speed2Hex).c_str());
  }
  else if (cmd == "GROUND") {
    if (stage == 0) {

      setMotorsSpeed(15000, -15000);
      stage = 1;
      T_ground = 0;
      T_ground_init = millis();
    }
    if ((millis() > (T_ground_init + T_ground + (1.1 / TRANSLATION_SPEED) * 1e3)) && (stage == 1)) {
      setMotorsSpeed(-15000, -15000);
      stage = 2;
      T_ground += (x / TRANSLATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + ((PI / 2) / ROTATION_SPEED) * 1e3)) && (stage == 2)) {
      setMotorsSpeed(15000, -15000);
      stage = 3;
      T_ground += ((PI / 2) / ROTATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + (4 / TRANSLATION_SPEED) * 1e3)) && (stage == 3)) {
      setMotorsSpeed(-15000, -15000);
      stage = 4;
      T_ground += (x / TRANSLATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + ((PI / 2) / ROTATION_SPEED) * 1e3)) && (stage == 4)) {
      setMotorsSpeed(15000, -15000);
      stage = 5;
      T_ground += ((PI / 2) / ROTATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + (2.2 / TRANSLATION_SPEED) * 1e3)) && (stage == 5)) {
      setMotorsSpeed(-15000, -15000);
      stage = 6;
      T_ground += (x / TRANSLATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + ((PI / 2) / ROTATION_SPEED) * 1e3)) && (stage == 6)) {
      setMotorsSpeed(15000, -15000);
      stage = 7;
      T_ground += ((PI / 2) / ROTATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + (4 / TRANSLATION_SPEED) * 1e3)) && (stage == 7)) {
      setMotorsSpeed(-15000, -15000);
      stage = 8;
      T_ground += (x / TRANSLATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground + ((PI / 2) / ROTATION_SPEED) * 1e3)) && (stage == 8)) {
      setMotorsSpeed(15000, -15000);
      stage = 9;
      T_ground += ((PI / 2) / ROTATION_SPEED) * 1e3;
    }
    if ((millis() > (T_ground_init + T_ground +(1.1 / TRANSLATION_SPEED) * 1e3) ) && (stage == 9)) {
      setMotorsSpeed(0, 0);
      stage = 10;
    }
  }

  else {
    Serial.println("[ERROR] Unknown command");
  }
}

String intToHexString(int value) {
  String result = "";
  for (int i = 0; i < 4; i++) {
    uint8_t byte_val = (uint8_t)((value >> (i * 8)) & 0xFF);
    if (byte_val < 0x10) result += "0";
    result += String(byte_val, HEX);
    if (i < 3) result += " ";
  }
  result.toUpperCase();
  return result;
}

void setMotorsSpeed(int speed1, int speed2) {
  String speed1Hex = intToHexString(speed1);
  String speed2Hex = intToHexString(speed2);

  Serial.println("[SPEED] Motor1: " + String(speed1) + " Motor2: " + String(speed2));
  send(0x601, ("23 FF 60 00 " + speed1Hex).c_str());
  send(0x602, ("23 FF 60 00 " + speed2Hex).c_str());
}
