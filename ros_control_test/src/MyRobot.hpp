#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <imu_sensor_controller/imu_sensor_controller.h>
#include "arduserial.h"
#include <iostream>
class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot():serialtest("/dev/ttyUSB0",7)
 {
   cmd[0]=0;
   cmd[1]=0;
   pos[0]=0;
   pos[1]=0;
   vel[0]=0;
   vel[1]=0;
   eff[0]=0;
   eff[1]=0;
   
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("JointA", &pos[0], &vel[0], &eff[0]); //pos vel eff outputs of the state message...
   jnt_state_interface.registerHandle(state_handle_a);
   hardware_interface::JointStateHandle state_handle_b("JointB", &pos[1], &vel[1], &eff[1]); //pos vel eff outputs of the state message...
   jnt_state_interface.registerHandle(state_handle_b);


   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("JointA"), &cmd[0]); //cmd is the commanded value depending on the controller.
   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("JointB"), &cmd[1]); //cmd is the commanded value depending on the controller.
   registerInterface(&jnt_state_interface);
   

   hardware_interface::ImuSensorHandle::Data data;
   data.name="ImuTest";
   data.frame_id="Imu";
   data.orientation=orientation;
   data.orientation_covariance=orientation_covariance;
   data.angular_velocity=&angular_velocity;
   data.angular_velocity_covariance=angular_velocity_covariance;
   data.linear_acceleration=&linear_acceleration;
   data.linear_acceleration_covariance=linear_acceleration_covariance;


   orientation_covariance[0]=1;
   orientation_covariance[1]=2;
   orientation_covariance[2]=3;
   orientation_covariance[3]=4;



   hardware_interface::ImuSensorHandle sensor_handle_imu(data);
   imu_interface.registerHandle(sensor_handle_imu);


   // connect and register the joint position interface
   jnt_pos_interface.registerHandle(pos_handle_a);
   jnt_vel_interface.registerHandle(pos_handle_a);
   jnt_eff_interface.registerHandle(pos_handle_a);
   
   jnt_pos_interface.registerHandle(pos_handle_b);
   jnt_vel_interface.registerHandle(pos_handle_b);
   jnt_eff_interface.registerHandle(pos_handle_b);

   
   
   
   registerInterface(&jnt_pos_interface);
   registerInterface(&jnt_vel_interface);
   registerInterface(&jnt_eff_interface);
   registerInterface(&imu_interface);

   serialtest.init();
}

  virtual ~MyRobot()
  {}

  void write()
  {
    std::cout<<"write "<<"  "<<pos[0]<<" "<<pos[1]<<std::endl;
    //std::cout<<"write "<<"  "<<cmd[0]<<" "<<cmd[1]<<std::endl;
    /*uint8_t data[7]={'0','2','3','2','0','5','\n'};
    int temp = (int)pos[0];
    for (int i=0;i<3;i++) {
      data[2-i] = temp%10+48;
      temp = temp/10;
    }
    temp = (int)pos[1];
    for (int i=0;i<3;i++) {
      data[5-i] = temp%10+48;
      temp = temp/10;
    }
    std::cout<<"data "<<"  "<<data[0]<<" "<<data[1]<<" "<<data[2]<<std::endl;
    serialtest.write(data);*/

    //int8_t data[5]={82,69,11,68,'\n'};
    uint8_t data[5]={255,0,36,2,36};
    serialtest.write(data, 5);
  }

  void read()
  {
    serialtest.read();
    pos[0] = serialtest.PWM_A;
    pos[1] = serialtest.PWM_B;
    std::cout<<"read c"<<"  "<<serialtest.PWM_A<<" "<<serialtest.PWM_B<<std::endl;
    std::cout<<"read d"<<"  "<<pos[0]<<" "<<pos[1]<<std::endl;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::ImuSensorInterface imu_interface;


  double orientation[4];                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
  double orientation_covariance[9];         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
  double angular_velocity;               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
  double angular_velocity_covariance[9];    ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
  double linear_acceleration;            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
  double linear_acceleration_covariance[9]; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)


  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  ArduSerial serialtest;
};
