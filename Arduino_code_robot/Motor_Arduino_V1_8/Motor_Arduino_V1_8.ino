// ============================================================
// Author      : Baptiste Poncet
// Date        : 22/07/2025
// File        : Motor_Arduino_V1_8.ino
// Version     : 1.8 (22/07/2025)
// Description : Code linking LoRa communication (UART) and motor (CANopen protocole)
//               with encoders informations/ Autonomous reading of encoders using TPDO
//               timely display, both direction
// ============================================================

#include <SPI.h>
#include <mcp_can.h>

#define CS_PIN 10
MCP_CAN CAN(CS_PIN);


String inputBuffer = "";


// Variables to prevent message corruption
bool serialBusy = false;
unsigned long lastSerialWrite = 0;
const unsigned long SERIAL_DELAY = 10;

float T_init;
float delta;
float last_pos;
float angle_tot;
bool first;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
int num_encoders = 2;

long unsigned int Id;
long angle;
//==================SETUP=================
void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial);
  if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    safeSerialPrint("CAN INIT FAILED");
    while (1);
  }

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

  first = true;
  angle_tot = 0;
  T_init = millis();

}
// ====================== SETUP END =========================

struct EncoderData {
  uint32_t can_id;
  int32_t last_position;
  int32_t total_position;
  bool first_read;
  bool indic;

  EncoderData(uint32_t id) : can_id(id), last_position(0), total_position(0), first_read(true) {}
};


EncoderData encoders[] = {
  EncoderData(0x381), // Moteur avec ID logique 0x01
  EncoderData(0x382)  // Moteur avec ID logique 0x02
};

void loop() {
  readEncoders_30cm(); //Send a message every 30 cm travelled by the robot

  if (Serial.available()) { // Process Serial input
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processSerialCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}



void Motor_init(byte id) {
  uint16_t cob_id = 0x600 + id;
  uint32_t tpdo_id = 0x180 + id;
  uint32_t mapped_id = 0x00000180 + id;

  send(cob_id, "2B 40 60 00 80 00 00 00"); delay(50);
  send(cob_id, "2B 40 60 00 06 00 00 00"); delay(50);
  send(cob_id, "2F 60 60 00 03 00 00 00"); delay(50);
  send(cob_id, "2B 40 60 00 07 00 00 00"); delay(50);

  sendSDO(cob_id, 0x1800, 0x01, 0x80000000 | tpdo_id, 4); delay(50);
  sendSDO(cob_id, 0x1A00, 0x00, 0x00, 1); delay(50);
  sendSDO(cob_id, 0x1A00, 0x01, 0x60640020, 4); delay(50);

  sendSDO(cob_id, 0x1A00, 0x00, 0x01, 1); delay(50);
  sendSDO(cob_id, 0x1800, 0x02, 0xFE, 1); delay(50);
  sendSDO(cob_id, 0x1800, 0x05, 100, 2); delay(50);
  sendSDO(cob_id, 0x1800, 0x01, mapped_id, 4); delay(50);


  send(cob_id, "2B 40 60 00 0F 00 00 00"); delay(50);
}


void safeSerialPrint(String message) {
  if (millis() - lastSerialWrite > SERIAL_DELAY && !serialBusy) {
    serialBusy = true;
    Serial.println(message);
    Serial.flush();
    lastSerialWrite = millis();
    serialBusy = false;
  }
}

void send(long id, const char *hexstr) {
  byte data[8] = {0};
  int index = 0;
  const char *p = hexstr;
  while (*p && index < 8) {
    while (*p == ' ') p++;  // skip spaces
    if (!*p) break;
    byte b = strtoul(p, NULL, 16);
    data[index++] = b;
    while (*p && *p != ' ') p++;
  }
  CAN.sendMsgBuf(id, 0, 8, data);

  if (millis() - lastSerialWrite > SERIAL_DELAY && !serialBusy) {
    serialBusy = true;
    String debugMsg = "Sent ID 0x" + String(id, HEX) + " : ";
    for (int i = 0; i < 8; i++) {
      if (data[i] < 0x10) debugMsg += "0";
      debugMsg += String(data[i], HEX) + " ";
    }
    Serial.println(debugMsg);
    Serial.flush();
    lastSerialWrite = millis();
    serialBusy = false;
  }
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
  if (CAN.sendMsgBuf(cob_id, 0, 8, data) != CAN_OK) {
    Serial.print("Erreur SDO pour ID ");
    Serial.println(cob_id, HEX);
  }
}

void readEncoders_30cm() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  int num_encoders = 2;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    for (int i = 0; i < num_encoders; i++) {
      EncoderData &enc = encoders[i];

      if (rxId == enc.can_id && len >= 4) {
        int32_t pos_raw = (int32_t)((rxBuf[3] << 24) | (rxBuf[2] << 16) | (rxBuf[1] << 8) | rxBuf[0]);
        int32_t pos = (pos_raw + 32767) * (360.0 / 65536);

        if (enc.first_read) {
          enc.last_position = pos;
          enc.first_read = false;
          return;
        }

        int32_t delta = abs(pos - enc.last_position);

        enc.total_position += delta;
        enc.last_position = pos;
        if (enc.total_position > 1565) {
          Serial.print("Encoder 0x");
          Serial.print(enc.can_id, HEX);
          Serial.print(" angle: ");
          Serial.println(enc.total_position);
          enc.total_position = 0;
          enc.last_position = 0;
        }

      }
    }
  }
}

void processSerialCommand(String cmd) {
  cmd.trim();
  unsigned long commandStartTime = millis();

  if (cmd.startsWith("SPEED")) {
    int sep1 = cmd.indexOf(' ');
    int sep2 = cmd.indexOf(' ', sep1 + 1);

    //Verify that the format is good
    if (sep1 == -1 || sep2 == -1) {
      safeSerialPrint("[ERROR] Wrong Format");
      return;
    }

    //Finding the velocity information
    String speed1Str = cmd.substring(sep1 + 1, sep2);
    String speed2Str = cmd.substring(sep2 + 1);

    int speed1 = speed1Str.toInt();
    int speed2 = speed2Str.toInt();

    String speed1Hex = intToHexString(speed1);
    String speed2Hex = intToHexString(speed2);

    safeSerialPrint("[SPEED] Motor1: " + String(speed1) + " Motor2: " + String(speed2));

    send(0x601, ("23 FF 60 00 " + speed1Hex).c_str());
    send(0x602, ("23 FF 60 00 " + speed2Hex).c_str());
    safeSerialPrint("[SPEED] Commands sent");
  }
  else {
    safeSerialPrint("[ERROR] Unknown command: " + cmd);
    safeSerialPrint("[INFO] Available commands: SPEED <s1> <s2>, ENCODERS, STATUS");
  }
}

String intToHexString(int value) { //Little endian
  String result = "";
  for (int i = 0; i < 4; i++) {
    uint8_t byte_val = (uint8_t)((value >> (i * 8)) & 0xFF);

    if (byte_val < 0x10) {
      result += "0";
    }
    result += String(byte_val, HEX);
    if (i < 3) {
      result += " ";
    }
  }
  result.toUpperCase();

  return result;
}
