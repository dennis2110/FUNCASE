#include "funcase_hw.h"

FuncaseRobot::FuncaseRobot() : serialimu("/dev/ttyACM0",5), serialdiff("/dev/ttyUSB0",5){
  for (int i=0;i<2;i++) {
    wheel_cmd[i] = 0;
    wheel_eff[i] = 0;
    wheel_pos[i] = 0;
    wheel_vel[i] = 0;
  }

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_a(
        "base_left_wheel_joint", &wheel_pos[0], &wheel_vel[0], &wheel_eff[0]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b(
        "base_right_wheel_joint", &wheel_pos[1], &wheel_vel[1], &wheel_eff[1]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_b);


  hardware_interface::JointHandle cmd_handle_a(
        jnt_state_interface.getHandle("base_left_wheel_joint"), &wheel_cmd[0]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle cmd_handle_b(
        jnt_state_interface.getHandle("base_right_wheel_joint"), &wheel_cmd[1]); //cmd is the commanded value depending on the controller.
  registerInterface(&jnt_state_interface);

  hardware_interface::ImuSensorHandle::Data data;
  data.name="ImuTest";
  data.frame_id="Imu";
  data.orientation=orientation;
  data.orientation_covariance=orientation_covariance;
  data.angular_velocity=angular_velocity;
  data.angular_velocity_covariance=angular_velocity_covariance;
  data.linear_acceleration=linear_acceleration;
  data.linear_acceleration_covariance=linear_acceleration_covariance;


  orientation_covariance[0]=1;
  orientation_covariance[1]=2;
  orientation_covariance[2]=3;
  orientation_covariance[3]=4;



  hardware_interface::ImuSensorHandle sensor_handle_imu(data);
  imu_interface.registerHandle(sensor_handle_imu);


  // connect and register the joint position interface
  jnt_pos_interface.registerHandle(cmd_handle_a);
  jnt_vel_interface.registerHandle(cmd_handle_a);
  jnt_eff_interface.registerHandle(cmd_handle_a);

  jnt_pos_interface.registerHandle(cmd_handle_b);
  jnt_vel_interface.registerHandle(cmd_handle_b);
  jnt_eff_interface.registerHandle(cmd_handle_b);


  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_eff_interface);
  registerInterface(&imu_interface);
}

FuncaseRobot::~FuncaseRobot(){

}

void FuncaseRobot::init(ros::NodeHandle *node){
  writediff[4] = 36;
}

void FuncaseRobot::read(){
  serialimu.read();
  serialdiff.read();
  for(int i=0;i<4;i++){
    orientation[i] = serialimu.orientation[i];
  }
  ROS_INFO("read imu data: %4.3f %4.3f %4.3f %4.3f",orientation[0],orientation[1],orientation[2],orientation[3]);
  ROS_INFO("read diff data: %03d %03d %03d %03d",serialdiff.raw_diff[0],serialdiff.raw_diff[1],serialdiff.raw_diff[2],serialdiff.raw_diff[3]);
}

void FuncaseRobot::write(){
  writediff[0] = 255;
  writediff[1] = 0;
  writediff[2] = 105;
  writediff[3] = 2;
  serialdiff.write(writediff,5);
  ROS_INFO("write diff data: %03d %03d %03d %03d",writediff[0],writediff[1],writediff[2],writediff[3]);
}
