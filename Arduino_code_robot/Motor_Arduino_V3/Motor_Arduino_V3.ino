// ============================================================
// Author      : Baptiste Poncet
// Date        : 13/08/2025
// File        : Motor_Arduino_V3.ino
// Version     : 3 (13/08/2025)
// Description : Code linking Jetson Orin (UART) and motor (CANopen protocole)
//               with encoders informations/ Autonomous reading of encoders using TPDO
//               timely display, both direction, ekf friendly
// ============================================================

#include <SPI.h>
#include <mcp_can.h>

#define CS_PIN 10
MCP_CAN CAN(CS_PIN);

String inputBuffer = "";
bool commandePrioritaireRecue = false;
unsigned long commandeTimestamp = 0;
const unsigned long commandeTimeout = 100;

const float TICKS_PER_REV = 65536.0;
const float WHEEL_DIAMETER = 0.13;    // m
const float WHEEL_CIRCUM = PI * WHEEL_DIAMETER;
const float WHEEL_BASE = 0.30;

float cumulative_distance = 0.0;
float accumulated_delta_L = 0.0;
float accumulated_delta_R = 0.0;
unsigned long lastNMT = 0;

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
  Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("[ERROR] CAN init failed");
    while (1);
  }

  CAN.init_Mask(0, 0, 0x7FF);
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(0, 0, 0x381);
  CAN.init_Filt(1, 0, 0x382);
  CAN.init_Filt(1, 1, 0x701);
  CAN.init_Filt(1, 2, 0x702);
  CAN.setMode(MCP_NORMAL);

  // Motors Pre-Op
  byte preOpNode1[] = {0x80, 0x01};
  byte preOpNode2[] = {0x80, 0x02};
  CAN.sendMsgBuf(0x000, 0, 2, preOpNode1); delay(50);
  CAN.sendMsgBuf(0x000, 0, 2, preOpNode2); delay(50);

  Motor_init(0x01);
  Motor_init(0x02);
  delay(100);

  // Motors Operational
  byte opNode1[] = {0x01, 0x01};
  byte opNode2[] = {0x01, 0x02};
  CAN.sendMsgBuf(0x000, 0, 2, opNode1); delay(50);
  CAN.sendMsgBuf(0x000, 0, 2, opNode2); delay(50);

  Serial.println("[INFO] Setup complete");
  cumulative_distance = 0.0;
}

// ==================== LOOP =============================
void loop() {
  checkSerialInput();
  readCANMessages();

  if (commandePrioritaireRecue && millis() - commandeTimestamp < commandeTimeout) {
    return; 
  }

  if (millis() - lastNMT > 3000) {
    byte opNode1[] = {0x01, 0x01};
    byte opNode2[] = {0x01, 0x02};
    CAN.sendMsgBuf(0x000, 0, 2, opNode1);
    CAN.sendMsgBuf(0x000, 0, 2, opNode2);
    lastNMT = millis();
  }

  updateOdometry();
}

// ==================== FONCTIONS ========================
void readCANMessages() {
  long unsigned int rxId;
  unsigned char len;
  unsigned char rxBuf[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == 0x701 || rxId == 0x702) {
      byte state = rxBuf[0];
      byte nodeId = rxId - 0x700;
      if (state != 0x05) {
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

  bool anyNew = encoders[0].indic || encoders[1].indic;
  if (anyNew || (now - last_encoder_check >= 50)) {
    last_encoder_check = now;
    float delta_L = encoders[0].indic ? -encoders[0].delta_meters : 0.0f;
    float delta_R = encoders[1].indic ?  encoders[1].delta_meters : 0.0f;
    if (fabs(delta_L) > 1e-4f || fabs(delta_R) > 1e-4f) {
      accumulated_delta_L += delta_L;
      accumulated_delta_R += delta_R;
    }
    encoders[0].delta_meters = encoders[1].delta_meters = 0.0f;
    encoders[0].indic = encoders[1].indic = false;
  }

  if (now - last_encoder_check >= 1500) {
    last_encoder_check = now;
    float delta_d = (accumulated_delta_L + accumulated_delta_R) / 2.0f;
    float delta_theta = (accumulated_delta_R - accumulated_delta_L) / WHEEL_BASE;
    cumulative_distance += fabs(delta_d);
    if (cumulative_distance >= 0.30f) {
      Serial.println("[DIST]");
      cumulative_distance = 0.0f;
    }
    Serial.print("EKF "); Serial.print(delta_d, 6); Serial.print(" "); Serial.println(delta_theta, 6);
    accumulated_delta_L = accumulated_delta_R = 0.0f;
  }
}

void Motor_init(byte id) {
  uint16_t cob_id = 0x600 + id;
  send(cob_id, "2B 40 60 00 80 00 00 00"); delay(30);
  send(cob_id, "2B 40 60 00 06 00 00 00"); delay(30);
  send(cob_id, "2F 60 60 00 03 00 00 00"); delay(30);
  send(cob_id, "2B 40 60 00 07 00 00 00"); delay(30);
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

// ==================== COMMANDE SERIE ===================
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
  if (cmd.startsWith("SPEED")) {
    int sep1 = cmd.indexOf(' ');
    int sep2 = cmd.indexOf(' ', sep1 + 1);
    if (sep1 == -1 || sep2 == -1) {
      Serial.println("[ERROR] Wrong Format");
      return;
    }
    int speed1 = cmd.substring(sep1 + 1, sep2).toInt();
    int speed2 = cmd.substring(sep2 + 1).toInt();
    Serial.println("[SPEED] " + String(speed1) + " " + String(speed2));
    send(0x601, ("23 FF 60 00 " + intToHexString(speed1)).c_str());
    send(0x602, ("23 FF 60 00 " + intToHexString(speed2)).c_str());
  } else {
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
