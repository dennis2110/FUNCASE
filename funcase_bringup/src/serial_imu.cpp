#include "serial_imu.h"

SerialIMU::SerialIMU(std::string port, size_t length) : ArduSerial (port, length){
  ArduSerial::init();
}

SerialIMU::~SerialIMU(){

}

void SerialIMU::read(){
  ArduSerial::read(raw_imu, 28);
  // get quaternion values
  int16_t w = (((0xff &(char)raw_imu[0]) << 8) | 0xff &(char)raw_imu[1]);
  int16_t x = (((0xff &(char)raw_imu[2]) << 8) | 0xff &(char)raw_imu[3]);
  int16_t y = (((0xff &(char)raw_imu[4]) << 8) | 0xff &(char)raw_imu[5]);
  int16_t z = (((0xff &(char)raw_imu[6]) << 8) | 0xff &(char)raw_imu[7]);

  double wf = w/16384.0;
  double xf = x/16384.0;
  double yf = y/16384.0;
  double zf = z/16384.0;

  orientation[0]= wf;
  orientation[1]= xf;
  orientation[2]= yf;
  orientation[3]= zf;

//  // get gyro values
//  int16_t gx = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);
//  int16_t gy = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);
//  int16_t gz = (((0xff &(char)input[data_packet_start + 14]) << 8) | 0xff &(char)input[data_packet_start + 15]);

//  double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
//  double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
//  double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

//  // get acelerometer values
//  int16_t ax = (((0xff &(char)input[data_packet_start + 16]) << 8) | 0xff &(char)input[data_packet_start + 17]);
//  int16_t ay = (((0xff &(char)input[data_packet_start + 18]) << 8) | 0xff &(char)input[data_packet_start + 19]);
//  int16_t az = (((0xff &(char)input[data_packet_start + 20]) << 8) | 0xff &(char)input[data_packet_start + 21]);
//  // calculate accelerations in m/s²
//  double axf = ax * (8.0 / 65536.0) * 9.81;
//  double ayf = ay * (8.0 / 65536.0) * 9.81;
//  double azf = az * (8.0 / 65536.0) * 9.81;

}
