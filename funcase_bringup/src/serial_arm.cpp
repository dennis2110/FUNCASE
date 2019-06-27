#include "serial_arm.h"

SerialArm::SerialArm(std::string port, size_t length) : ArduSerial (port, length){
  ArduSerial::init();
}

SerialArm::~SerialArm(){}

void SerialArm::read(){
  ArduSerial::read(raw_arm,5);
}
