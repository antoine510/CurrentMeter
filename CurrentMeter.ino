#include <Wire.h>
#include <EEPROM.h>

constexpr uint8_t I2C_ADDR = 0b1001000;
constexpr uint8_t MAGIC_NUMBER = 0x42;

enum Pins {
  PIN_TXEN = 2  // PD2
};

enum CommandID : uint8_t {
  MAGIC = 0x0,            // Returns the magic number (1 byte unsigned)
  READ_CURRENT_MA = 0x1   // Returns the averaged current since last read event in milliamps (4 bytes signed)
};

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(PIN_TXEN, OUTPUT);
  digitalWrite(PIN_TXEN, LOW);
}

int32_t GetCurrent_mA() {
  uint8_t buf[2];
  Wire.requestFrom(I2C_ADDR, (uint8_t)2);
  Wire.readBytes((char*)buf, 2);
  int16_t current_count = buf[0] << 8 | buf[1];
  return (int32_t)(current_count + 33) * 31 / 10;  // Calibration constants
}

struct SerialData {
  int32_t sumCurrent_mA = 0;
  uint8_t crc;
} sdata;
uint16_t sampleCount = 0;

constexpr unsigned long stateUpdatePeriod = 1000ul;

void updateState() {
  sdata.sumCurrent_mA += GetCurrent_mA();
  sampleCount++;
}

void runCommand(CommandID command) {
  switch(command) {
  case MAGIC:
    SendRS485(&MAGIC_NUMBER, sizeof(MAGIC_NUMBER));
    break;
  case READ_CURRENT_MA:
    sdata.sumCurrent_mA /= sampleCount;
    sdata.crc = crc((uint8_t*)&sdata, sizeof(SerialData) - 1);
    SendRS485((uint8_t*)&sdata, sizeof(SerialData));
    sdata.sumCurrent_mA = 0;
    sampleCount = 0;
    break;
  }
}

void SendRS485(uint8_t* data, size_t len) {
  PORTD |= 0x04;
  Serial.write(data, len);
  Serial.flush();
  PORTD &= 0xfb;
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
  static unsigned long nextStateUpdate = 0;
  if(millis() > nextStateUpdate) {
    nextStateUpdate += stateUpdatePeriod;
    updateState();
  }

  while(Serial.available()) {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
  }
}

uint8_t crc(uint8_t *data, uint8_t len) {
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
