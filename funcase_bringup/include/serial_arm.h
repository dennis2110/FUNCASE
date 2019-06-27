#ifndef SERIAL_ARM_H
#define SERIAL_ARM_H

#include "arduserial.h"

class SerialArm : public ArduSerial{
public:
 SerialArm(std::string port, size_t length);
 ~SerialArm();
 void read();

private:

public:
 uint8_t raw_arm[1];
private:

};

#endif // SERIAL_ARM_H
