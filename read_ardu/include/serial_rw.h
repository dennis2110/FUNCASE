#ifndef SERIAL_RW_H
#define SERIAL_RW_H
#include "serial/serial.h"
#include <ros/ros.h>

class ArduSerial{
public:
  ArduSerial(std::string port, size_t length);
  ~ArduSerial();
  void init();
  void read();
  void write();
private:

public:

private:
  serial::Serial _ser;
  std::string _port;
  size_t _length;
};

#endif // SERIAL_RW_H