# corolla_can-stm32


SPI Interface 1 (MOSI: PB15, MISO: PB14, SCK: PB10, CS: PB9) is for the MCP2515 CAN Board

SPI Interface 2 (MOSI: PA7, MISO: PA6, SCK: PA5, CS:PA4) is for the AS5048A Magnetic Rotary Encoder

The steering angle booster measures the current steering wheel angle that is reported by the car, and then marks the current location of the ZSS sensor and then will compute and offset which is then used as the steering angle fraction. Theoretically it is accurate to 0.02 of a degree. In reality it's probably more like 0.05-0.1
