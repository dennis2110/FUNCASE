#ifndef SERIAL_DIFF_H
#define SERIAL_DIFF_H

#include "arduserial.h"

class SerialDiff : public ArduSerial{
public:
  SerialDiff(std::string port, size_t length);
  ~SerialDiff();
  void read();

private:

public:
  uint8_t raw_diff[4];
private:


};

#endif // SERIAL_DIFF_H
