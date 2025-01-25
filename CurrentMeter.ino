#include <Wire.h>
#include <EEPROM.h>

constexpr uint8_t I2C_ADDR = 0b1001000;
constexpr unsigned long stateUpdatePeriod = 1000ul;

enum Pins {
  PIN_TXEN = 2  // PD2
};

enum CommandID : uint8_t {
  MAGIC = 0x0,            // Returns the magic number (1 byte unsigned)
  READ_CURRENT_MA = 0x1   // Returns the averaged current since last read event in milliamps (4 bytes signed)
};


extern "C" void __attribute__((naked, used, section (".init3"))) init3() {
  DDRD = _BV(DDD2);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

int32_t GetCurrent_mA() {
  uint8_t buf[2];
  Wire.requestFrom(I2C_ADDR, (uint8_t)2);
  Wire.readBytes((char*)buf, 2);
  int16_t current_count = buf[0] << 8 | buf[1];
  return (int32_t)(current_count + 33) * 31 / 10;  // Calibration constants
}

int32_t sumCurrent_mA = 0;
uint16_t sampleCount = 0;

void updateState() {
  sumCurrent_mA += GetCurrent_mA();
  sampleCount++;
}

void runCommand(CommandID command) {
  constexpr uint8_t MAGIC_NUMBER = 0x42;
  switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER), false);
    break;
  case READ_CURRENT_MA:
    sumCurrent_mA /= sampleCount;
    SendRS485((uint8_t*)&sumCurrent_mA, sizeof(sumCurrent_mA), true);
    sumCurrent_mA = 0;
    sampleCount = 0;
    break;
  }
}

void SendRS485(uint8_t* data, size_t len, bool withCRC) {
  const uint8_t msg_crc = crc(data, len);
  PORTD |= _BV(PORTD2);
  Serial.write(data, len);
  if(withCRC) Serial.write(msg_crc);
  Serial.flush();
  PORTD &= ~_BV(PORTD2);
}

enum SerialSeq : uint8_t {
  MAGIC1, MAGIC2, IDENTITY, COMMAND
} serial_state = MAGIC1;

uint8_t updateSerialState(uint8_t byte) {
  static constexpr const uint8_t cmd_magic[] = {0x4f, 0xc7};
  switch(serial_state) {
    case MAGIC1: return byte == cmd_magic[0] ? MAGIC2 : MAGIC1;
    case MAGIC2: return byte == cmd_magic[1] ? IDENTITY : MAGIC1;
    case IDENTITY: return byte == EEPROM.read(0) ? COMMAND : MAGIC1;
    case COMMAND: runCommand((CommandID)byte); return MAGIC1;
  }
}


void loop() {
  static unsigned long lastStateUpdate = 0;
  auto now = millis();
  if(now - lastStateUpdate > stateUpdatePeriod) {
    lastStateUpdate = now;
    updateState();
  }

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
  }
}

uint8_t crc(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t b = 0; b < len; ++b) {
    crc ^= data[b];
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;	// SMBUS CRC8
      else
        crc = crc << 1;
    }
  }
  return crc;
}
