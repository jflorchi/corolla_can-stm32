#include <Arduino.h>
#include <mcp2515.h>
#include <can.h>
#include <SPI.h>
#include <AS5048A.h>

/**
 * Function definitions
 */
void writeMsgEPS(uint16_t id, uint8_t *msg, uint8_t len, bool checksum);
void writeMsgCAR(uint16_t id, uint8_t *msg, uint8_t len, bool checksum);
void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg);
int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr);
bool isAllowedMessage(uint16_t messageId);
void initCarCan();
void initEpsCan();

/**
 * constant messages
 */
uint8_t GEAR_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};

uint8_t MSG7[2] = {0x06, 0x00};
uint8_t MSG8[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00};
uint8_t MSG10[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG11[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG12[8] = {0x66, 0x06, 0x08, 0x0a, 0x02, 0x00, 0x00, 0x00};
uint8_t MSG13[8] = {0x1C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG15[8] = {0x05, 0xea, 0x1b, 0x08, 0x00, 0x00, 0xc0, 0x9f};

uint8_t ANGLE[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * Variable messages
 */
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};
uint8_t WHEEL_SPEEDS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t ZSS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

bool openEnabled = false;
uint16_t setSpeed = 0x00;
bool blinkerRight = false, blinkerLeft = false;

uint16_t zero = 6871;

uint32_t startedLoop = -1;
uint8_t loopCounter = 0;

AS5048A angleSensor(PB9);
MCP2515 carCan(PA15, PA7, PA6, PA5);
MCP2515 epsCan(PA4, PA7, PA6, PA5);

const uint8_t ALLOWED_MESSAGES_SIZE = 5;
uint16_t ALLOWED_MESSAGES[ALLOWED_MESSAGES_SIZE] = {0xb4, 0x3b1, 0x2c1, 0x399, 0x24};

const uint8_t numReadings = 12;
uint8_t readIndex = 0;
int readings[numReadings];
int total;
int average;

void setup() {
  Serial.begin(115200);
  angleSensor.init();
  angleSensor.setZeroPosition(zero);
  initCarCan();
  initEpsCan();
}

void loop() {
  if (loopCounter == 1001) {
    loopCounter = 0;
  }

  total -= readings[readIndex];
  readings[readIndex] = angleSensor.getRotation();
  total += readings[readIndex++];
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // When the RX buffer fills up it will flip a bit and stop listening for messages on that register. It has to be manually cleared.
  if (carCan.checkRXnOVR()) {
    carCan.clearRXnOVR();
  }
  if (epsCan.checkRXnOVR()) {
    epsCan.clearRXnOVR();
  }

  // If RX / TX errors hit 128 then they go into a fail mode, if they do (even though they shouldn't) just re-init them
  if (epsCan.errorCountRX() > 127 || epsCan.errorCountTX() > 127) {
    initEpsCan();
  }
  if (carCan.errorCountRX() > 127 || carCan.errorCountTX() > 127) {
    initCarCan();
  }

  // READ FROM CAR CAN BUS
  struct can_frame frame;
  MCP2515::ERROR result;
  while ((result = carCan.readMessage(&frame)) == MCP2515::ERROR_OK) {
    if (frame.can_id == 0xb0) {
        uint8_t dat[8];
        WHEEL_SPEEDS[0] = frame.data[0] + 0x1a;
        WHEEL_SPEEDS[1] = frame.data[1] + 0x6f;
        WHEEL_SPEEDS[2] = frame.data[2] + 0x1a;
        WHEEL_SPEEDS[3] = frame.data[3] + 0x6f;
    } else if (frame.can_id == 0xb2) {
        WHEEL_SPEEDS[4] = frame.data[0] + 0x1a;
        WHEEL_SPEEDS[5] = frame.data[1] + 0x6f;
        WHEEL_SPEEDS[6] = frame.data[2] + 0x1a;
        WHEEL_SPEEDS[7] = frame.data[3] + 0x6f;
    } else if (frame.can_id == 0x399) {
        openEnabled = (frame.data[1] & 0x2) == 2;
    }

    // FORWARD MESSGES TO THE EPS CAN BUS
    if (isAllowedMessage(frame.can_id)) {
        epsCan.sendMessage(&frame);
      }
  } 

  // READ FROM EPS CAN BUS
  while ((result = epsCan.readMessage(&frame)) == MCP2515::ERROR_OK) {
    if (frame.can_id == 0x260 || frame.can_id == 0x262 || frame.can_id == 0x394) { // 916
      carCan.sendMessage(&frame); // forward every message from the EPS to the car
    }
  }

  if (loopCounter == 0 || loopCounter % 5 == 0) {
    /* code */
  }
  

  // // 100 Hz:
  if (loopCounter == 0 || loopCounter % 12 == 0) {
    writeMsgEPS(0x25, ANGLE, 8, true);
    writeMsgEPS(0xaa, WHEEL_SPEEDS, 8, false);
    writeMsgEPS(0x3bc, GEAR_MSG, 8, false);
    writeMsgCAR(0x3bc, GEAR_MSG, 8, false);
    writeMsgEPS(0x1c4, MSG15, 8, false);
    writeMsgEPS(0x414, MSG8, 7, false);
    writeMsgEPS(0x489, MSG10, 7, false);
    writeMsgEPS(0x48a, MSG11, 7, false);
    writeMsgEPS(0x48b, MSG12, 8, false);
    writeMsgEPS(0x4d3, MSG13, 8, false);

    if (!angleSensor.error()) {
      int raw = total / numReadings;
      ZSS[0] = raw >> 24;
      ZSS[1] = raw >> 16;
      ZSS[2] = raw >> 8;
      ZSS[3] = raw;
      writeMsgCAR(0x23, ZSS, 8, false);  
    }
  }
  
  // 40 Hz:
  if (loopCounter == 0 || loopCounter % 25 == 0) {
    writeMsgEPS(0x367, MSG7, 2, false);
  }

  // 1 Hz:
  if (loopCounter == 0) {
    STEERING_LEVER_MSG[3] = (blinkerLeft << 5) & 0x20 | (blinkerRight << 4) & 0x10;
    writeMsgCAR(0x614, STEERING_LEVER_MSG, 8, true);
  }

  delay(1);
  loopCounter++;
}

void initEpsCan() {
  epsCan.init();
  epsCan.reset();
  epsCan.setBitrate(CAN_500KBPS, MCP_16MHZ);
  epsCan.setNormalMode();
}

void initCarCan() {
  carCan.init();
  carCan.reset();
  carCan.setBitrate(CAN_500KBPS, MCP_16MHZ);
  carCan.setNormalMode();
}

bool isAllowedMessage(uint16_t messageId) {
  if (messageId >= 0x700) {
    return true;
  }
  for (uint8_t i = 0; i < ALLOWED_MESSAGES_SIZE; i++) {
    if (messageId == ALLOWED_MESSAGES[i]) {
      return true;
    }
  }
  return false;
}

void writeMsgEPS(uint16_t id, uint8_t *msg, uint8_t len, bool checksum) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    if (checksum) {
        attachChecksum(id, len, msg);
    }
    for (int i = 0; i < len; i++) {
      frame.data[i] = msg[i];
    }
    epsCan.sendMessage(&frame);
}

void writeMsgCAR(uint16_t id, uint8_t *msg, uint8_t len, bool checksum) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = len;
    if (checksum) {
        attachChecksum(id, len, msg);
    }
    for (int i = 0; i < len; i++) {
      frame.data[i] = msg[i];
    }
    carCan.sendMessage(&frame);
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
