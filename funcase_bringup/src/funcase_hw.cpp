#include "funcase_hw.h"

FuncaseRobot::FuncaseRobot() : /*serialimu("/dev/ttyUSB1",5),*/ serialdiff("/dev/ttyUSB0",5){
  // init param
  for (int i=0;i<2;i++) {
    wheel_cmd[i] = 0;
    wheel_eff[i] = 0;
    wheel_pos[i] = 0;
    wheel_vel[i] = 0;
  }
  for (int i=0;i<3;i++) {
    angular_velocity[i] = 0.0;
    linear_acceleration[i] = 0.0;
  }
  for (int i=0;i<9;i++) {
    orientation_covariance[i] = 0.0;
    angular_velocity_covariance[i] = 0.0;
    linear_acceleration_covariance[i] = 0.0;
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
  m_track_line_pub = node->advertise<std_msgs::UInt8MultiArray>("/track_line_sensor",100);

  for(int i=0;i<4;i++){
    writediff[i]=0;
  }
  writediff[4] = 10;
  ROS_INFO("funcasebot init");
}

void FuncaseRobot::read(){
  //serialimu.read();
  serialdiff.read();
  //for(int i=0;i<4;i++){
  //  orientation[i] = serialimu.orientation[i];
  //}

  cny70[0] = serialdiff.raw_diff[0];
  cny70[1] = serialdiff.raw_diff[1];
  cny70[2] = serialdiff.raw_diff[2];
  cny70[3] = serialdiff.raw_diff[3];
  //ROS_INFO("read imu data: %4.3f %4.3f %4.3f %4.3f",orientation[0],orientation[1],orientation[2],orientation[3]);
  //ROS_INFO("read diff data: %4.1f %4.1f Mode: %1.1f %1.1f",wheel_vel[0],wheel_vel[1],wheel_eff[0],wheel_eff[1]);
  publish_sensor_data();
}

void FuncaseRobot::write(){
  wheelcmd2writediff(wheel_cmd[0],0); //set writediff[0 and 1] from wheel_cmd[0]
  wheelcmd2writediff(wheel_cmd[1],2); //set writediff[2 and 3] from wheel_cmd[0]
  ///ROS_INFO("diff data: %03d %03d %03d %03d",writediff[0],writediff[1],writediff[2],writediff[3]);


  serialdiff.write(writediff,5);
}

void FuncaseRobot::wheelcmd2writediff(double cmd,int n){
  uint8_t temp;
  if(cmd == 0.0){
    writediff[n] = 0;
    writediff[n+1] = 3;
  }else if (cmd < 0) {
    if (cmd <-255){
      temp = 255;
    }else{
      temp = static_cast<uint8_t>(-cmd);
    }
    writediff[n] = temp;
    writediff[n+1] = 2;
  }else if (cmd > 0) {
    if (cmd >255){
      temp = 255;
    }else{
      temp = static_cast<uint8_t>(cmd);
    }
    writediff[n] = temp;
    writediff[n+1] = 1;
  }
}

void FuncaseRobot::publish_sensor_data(){
  std_msgs::UInt8MultiArray sensor_msg;
  for (int i=0;i<4;i++) {
    sensor_msg.data.push_back(cny70[i]);
  }
  m_track_line_pub.publish(sensor_msg);
}
