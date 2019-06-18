#include "serial_diff.h"

SerialDiff::SerialDiff(std::string port, size_t length) : ArduSerial (port, length){
  ArduSerial::init();
}

SerialDiff::~SerialDiff(){}

void SerialDiff::read(){
  ArduSerial::read(raw_diff, SENSOR_REG_COUNT + 4);
}
