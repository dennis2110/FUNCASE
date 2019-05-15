#include "serial_rw.h"

ArduSerial::ArduSerial(std::string port, size_t length){
  _port = port;
  _length = length;
}

ArduSerial::~ArduSerial(){

}

void ArduSerial::init(){
  _ser.setPort(_port);
  _ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  _ser.setTimeout(to);
  _ser.open();
}

void ArduSerial::read(){

}

void ArduSerial::write(){
  //write serial
  uint8_t TwoPWM[7] = {'1','2','3','2','5','5','\n'};
  int size_write = _ser.write(TwoPWM, 7);
  std::cout << "write: " << size_write <<std::endl;

}
