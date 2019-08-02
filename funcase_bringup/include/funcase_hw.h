#ifndef FUNCASE_HW_H
#define FUNCASE_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <imu_sensor_controller/imu_sensor_controller.h>
#include <iostream>
#include "serial_imu.h"
#include "serial_diff.h"
#include <std_msgs/UInt8MultiArray.h>

#define NORMALIZE_CNY70
#define SENSOR_REG_COUNT (7)

class FuncaseRobot : public hardware_interface::RobotHW
{
public:
  FuncaseRobot();
  ~FuncaseRobot();

  ros::Time getTime() const {return ros::Time::now(); }
  ros::Duration getPeriod() const {return ros::Duration(0.01); }

  void init(ros::NodeHandle* node);
  void read();
  void write();
private:
  void wheelcmd2writediff(double cmd, int n);
  void publish_sensor_data();
  uint8_t normalize(uint8_t value, uint8_t max, uint8_t min);
public:

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::ImuSensorInterface imu_interface;

  double orientation[4];                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
  double orientation_covariance[9];         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
  double angular_velocity[3];                ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
  double angular_velocity_covariance[9];    ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
  double linear_acceleration[3];            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
  double linear_acceleration_covariance[9]; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)

  double wheel_cmd[2];
  double wheel_pos[2];
  double wheel_vel[2];
  double wheel_eff[2];

  uint8_t cny70[SENSOR_REG_COUNT];
  //SerialIMU serialimu;
  SerialDiff serialdiff;

  uint8_t writediff[5];

  // Publisher who publish the Tracking line sensor
  ros::Publisher m_track_line_pub;
};

#endif // FUNCASE_HW_H
