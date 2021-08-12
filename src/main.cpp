#include <Arduino.h>
#include <mcp2515.h>
#include <can.h>
#include <SPI.h>

/**
 * Function definitions
 */
void recv(int packetSize);
void writeMsg(uint16_t id, uint8_t *msg, uint8_t len, bool checksum);
void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg);
int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr);

/**
 * constant messages
 */
uint8_t GEAR_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};
uint8_t PRE_COL[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8c};
uint8_t PRE_COL_2[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4f};
uint8_t BRAKE_MOD[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8};
uint8_t STEER_ANGLE[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t CAMERA[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

uint8_t MSG1[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38};
uint8_t MSG2[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
uint8_t MSG3[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
uint8_t MSG4[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
uint8_t MSG5[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
uint8_t MSG6[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t MSG7[2] = {0x06, 0x00};
uint8_t MSG8[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00};
uint8_t MSG9[3] = {0x24, 0x20, 0xB1};
uint8_t MSG10[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG11[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG12[8] = {0x66, 0x06, 0x08, 0x0a, 0x02, 0x00, 0x00, 0x00};
uint8_t MSG13[8] = {0x1C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG14[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00};
uint8_t MSG15[8] = {0x05, 0xea, 0x1b, 0x08, 0x00, 0x00, 0xc0, 0x9f};
uint8_t MSG16[8] = {0x08, 0x07, 0x07, 0x06, 0x70, 0xf8, 0x00, 0x4f};
uint8_t MSG17[2] = {0x00, 0x00};
uint8_t MSG18[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG19[4] = {0x00, 0x00, 0x26, 0x00};
uint8_t MSG20[8] = {0x76, 0x18, 0x26, 0x01, 0x00, 0x00, 0x00, 0xb9};
uint8_t MSG21[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00};
uint8_t MSG22[8] = {0x02, 0x00, 0x01, 0xfd, 0x42, 0x03, 0x80, 0xf1};
uint8_t MSG23[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00};
uint8_t MSG24[8] = {0x28, 0x00, 0x60, 0x01, 0x0a, 0x00, 0xa3, 0xa0};
uint8_t MSG25[6] = {0xf4, 0x01, 0x90, 0x83, 0x00, 0x37};
uint8_t MSG26[4] = {0x00, 0x00, 0x00, 0x46};
uint8_t MSG27[8] = {0x00, 0x00, 0x08, 0x12, 0x01, 0x31, 0x9c, 0x51};
uint8_t MSG28[7] = {0x00, 0x1e, 0x00, 0x00, 0x00, 0x80, 0x07};
uint8_t MSG29[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8c};
uint8_t MSG30[8] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x50};
uint8_t MSG31[7] = {0x00, 0x00, 0x00, 0x80, 0xfc, 0x00, 0x08};
uint8_t MSG32[7] = {0x00, 0x72, 0x07, 0xff, 0x09, 0xfe, 0x00};
uint8_t MSG33[8] = {0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * Variable messages
 */
uint8_t LKAS_MSG[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};
uint8_t WHEEL_SPEEDS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t ZSS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

bool openEnabled = false;
uint16_t setSpeed = 0x00;
bool blinkerRight = false, blinkerLeft = false;

uint8_t loopCounter = 0;

void recv(int packetSize) {
    // long id = can.packetId();
    // Serial.println(id);
    // if (id == 0xb0) {
    //     uint8_t dat[8];
    //     WHEEL_SPEEDS[0] = can.read() + 0x1a;
    //     WHEEL_SPEEDS[1] = can.read() + 0x6f;
    //     WHEEL_SPEEDS[2] = can.read() + 0x1a;
    //     WHEEL_SPEEDS[3] = can.read() + 0x6f;
    // } else if (id == 0xb2) {
    //     WHEEL_SPEEDS[4] = can.read() + 0x1a;
    //     WHEEL_SPEEDS[5] = can.read() + 0x6f;
    //     WHEEL_SPEEDS[6] = can.read() + 0x1a;
    //     WHEEL_SPEEDS[7] = can.read() + 0x6f;
    // } else if (id == 0x399) {
    //     can.read();
    //     openEnabled = (can.read() & 0x2) == 2;
    // } else if (id == 0x25) {

    // }
}

MCP2515 can(PB9);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(PC13, OUTPUT);
  pinMode(PC13, LOW);

  can.reset();
  can.setBitrate(CAN_500KBPS, MCP_16MHZ);
  can.setNormalMode();
}

void loop() {
  if (loopCounter == 1001) {
    loopCounter = 0;
  }

  struct can_frame frame; // will need a while loop
  uint8_t val = can.readMessage(&frame);
  if (val == MCP2515::ERROR_OK) {
    Serial.println(frame.can_id);
    digitalWrite(PC13, HIGH);
  } else {
    Serial.println(val);
    digitalWrite(PC13, LOW);
  }

  // 100 Hz:
  if (loopCounter == 0 || loopCounter % 10 == 0) {
    writeMsg(0x1c4, MSG15, 8, false);
    writeMsg(0xaa, WHEEL_SPEEDS, 8, false);
    writeMsg(0x130, MSG1, 7, false);
    writeMsg(0x414, MSG8, 7, false);
    writeMsg(0x466, MSG9, 3, false);
    writeMsg(0x489, MSG10, 7, false);
    writeMsg(0x48a, MSG11, 7, false);
    writeMsg(0x48b, MSG12, 8, false);
    writeMsg(0x4d3, MSG13, 8, false);
    writeMsg(0x3bc, GEAR_MSG, 8, false);
    writeMsg(0x3bb, MSG19, 4, false);
    writeMsg(0x4cb, MSG33, 8, false);

    // ZSS[0] = getSteerFraction();
    writeMsg(0x23, ZSS, 8, false);
  }
  
  // 50 Hz:
  if (loopCounter == 0 || loopCounter % 20 == 0) {
    writeMsg(0x3d3, MSG17, 2, false);
    writeMsg(0x4ac, MSG24, 8, false);
  }
  
  // 40 Hz:
  if (loopCounter == 0 || loopCounter % 25 == 0) {
    writeMsg(0x367, MSG7, 2, false);
  }
  
  // 20 Hz:
  if (loopCounter == 0 || loopCounter % 50 == 0) {
    writeMsg(0x3f9, MSG20, 8, false);
    writeMsg(0x365, MSG31, 7, false);
    writeMsg(0x366, MSG32, 7, false);
  }
  
  // 7 Hz
  if (loopCounter == 0 || loopCounter % 142 == 0) {
    writeMsg(0x160, MSG27, 8, false);
    writeMsg(0x161, MSG28, 7, false);
  }
  
  // 5 Hz
  if (loopCounter == 0 || loopCounter % 200 == 0) {
    writeMsg(0x240, MSG2, 7, false);
    writeMsg(0x241, MSG3, 7, false);
    writeMsg(0x244, MSG4, 7, false);
    writeMsg(0x245, MSG5, 7, false);
    writeMsg(0x248, MSG6, 7, false);
    writeMsg(0x344, MSG30, 8, false);
  }
  
  // 3 Hz:
  if (loopCounter == 0 || loopCounter % 333 == 0) {
    writeMsg(0x128, MSG25, 6, false);
    writeMsg(0x283, MSG29, 7, false);

    PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
    PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
    writeMsg(0x1d2, PCM_CRUISE_MSG, 8, true);
    
    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
    writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);
  }
  
  // 2 Hz:    
  if (loopCounter == 0 || loopCounter % 500 == 0) {
    writeMsg(0x141, MSG26, 4, false);
  }
  
  // 1 Hz:
  if (loopCounter == 0) {
    STEERING_LEVER_MSG[3] = (blinkerLeft << 5) & 0x20 | (blinkerRight << 4) & 0x10;
    writeMsg(0x614, STEERING_LEVER_MSG, 8, true);
  }

  delay(1);
  loopCounter++;
}

void writeMsg(uint16_t id, uint8_t *msg, uint8_t len, bool checksum) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    if (checksum) {
        attachChecksum(id, len, msg);
    }
    for (int i = 0; i < len; i++) {
      frame.data[i] = msg[i];
    }
    can.sendMessage(&frame);
}

void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg) {
    msg[len -1] = getChecksum(msg, len - 1, id);
}

int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (msg[ii]);
    }
    return checksum;
}
