#ifndef SERIAL_IMU_H
#define SERIAL_IMU_H

#include "arduserial.h"

class SerialIMU : public ArduSerial{
public:
  SerialIMU(std::string port, size_t length);
  ~SerialIMU();
  void read();

private:

public:
  uint8_t raw_imu[24];
  double orientation[4];                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
  //double angular_velocity[3];                ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
  //double linear_acceleration[3];            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)

private:

};

#endif // SERIAL_IMU_H
