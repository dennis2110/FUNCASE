#include "funcase_hw.h"

FuncaseRobot::FuncaseRobot() : /*serialimu("/dev/ttyUSB1",5),*/ serialdiff("/dev/ttyUSB0",5){
  for (int i=0;i<2;i++) {
    wheel_cmd[i] = 0;
    wheel_eff[i] = 0;
    wheel_pos[i] = 0;
    wheel_vel[i] = 0;
  }

  // connect and register the joint state interface
  //wheel
  hardware_interface::JointStateHandle state_handle_a(
        "base_left_wheel_joint", &wheel_pos[0], &wheel_vel[0], &wheel_eff[0]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b(
        "base_right_wheel_joint", &wheel_pos[1], &wheel_vel[1], &wheel_eff[1]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_b);

  //arm
  hardware_interface::JointStateHandle state_handle_right_arm_base(
        "right_arm_base_link_joint", &arm_pos[0], &arm_vel[0], &arm_eff[0]);
  jnt_state_interface.registerHandle(state_handle_right_arm_base);

  hardware_interface::JointStateHandle state_handle_right_arm_2(
        "right_arm_2_link_joint", &arm_pos[1], &arm_vel[1], &arm_eff[1]);
  jnt_state_interface.registerHandle(state_handle_right_arm_2);

  hardware_interface::JointStateHandle state_handle_right_arm_3(
        "right_arm_3_link_joint", &arm_pos[2], &arm_vel[2], &arm_eff[2]);
  jnt_state_interface.registerHandle(state_handle_right_arm_3);

  hardware_interface::JointStateHandle state_handle_right_arm_4(
        "right_arm_4_link_joint", &arm_pos[3], &arm_vel[3], &arm_eff[3]);
  jnt_state_interface.registerHandle(state_handle_right_arm_4);

  hardware_interface::JointStateHandle state_handle_right_arm_5(
        "right_arm_5_link_joint", &arm_pos[4], &arm_vel[4], &arm_eff[4]);
  jnt_state_interface.registerHandle(state_handle_right_arm_5);

  registerInterface(&jnt_state_interface);

  // register thr joint command
  //wheel -> connect and register the joint effort interface
  hardware_interface::JointHandle cmd_handle_a(
        jnt_state_interface.getHandle("base_left_wheel_joint"), &wheel_cmd[0]); //cmd is the commanded value depending on the controller.
  jnt_eff_interface.registerHandle(cmd_handle_a);
  hardware_interface::JointHandle cmd_handle_b(
        jnt_state_interface.getHandle("base_right_wheel_joint"), &wheel_cmd[1]); //cmd is the commanded value depending on the controller.
  jnt_eff_interface.registerHandle(cmd_handle_b);

  registerInterface(&jnt_eff_interface);

  //arm -> connect and register the joint position interface
  hardware_interface::JointHandle pos_cmd_handle_right_arm_base(
        jnt_state_interface.getHandle("right_arm_base_link_joint"), &arm_cmd[0]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_base);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_2(
        jnt_state_interface.getHandle("right_arm_2_link_joint"), &arm_cmd[1]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_2);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_3(
        jnt_state_interface.getHandle("right_arm_3_link_joint"), &arm_cmd[2]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_3);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_4(
        jnt_state_interface.getHandle("right_arm_4_link_joint"), &arm_cmd[3]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_4);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_5(
        jnt_state_interface.getHandle("right_arm_5_link_joint"), &arm_cmd[4]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_5);

  registerInterface(&jnt_pos_interface);

  //imu -> connect and register the imu interface
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
//  wheel_vel[0]= static_cast<double>(serialdiff.raw_diff[0]); //wheel_vel is PWM of wheel
//  wheel_vel[1]= static_cast<double>(serialdiff.raw_diff[2]);
//  wheel_eff[0]= static_cast<double>(serialdiff.raw_diff[1]); //wheel_eff is mode of wheel
//  wheel_eff[1]= static_cast<double>(serialdiff.raw_diff[3]);
  cny70[0] = serialdiff.raw_diff[0];
  cny70[1] = serialdiff.raw_diff[1];
  cny70[2] = serialdiff.raw_diff[2];
  cny70[3] = serialdiff.raw_diff[3];
  //ROS_INFO("read imu data: %4.3f %4.3f %4.3f %4.3f",orientation[0],orientation[1],orientation[2],orientation[3]);
  //ROS_INFO("read diff data: %4.1f %4.1f Mode: %1.1f %1.1f",wheel_vel[0],wheel_vel[1],wheel_eff[0],wheel_eff[1]);
  publish_sensor_data();
}

void FuncaseRobot::write(){
  //wheel_cmd[0] = 50;
  //wheel_cmd[1] = -120;
  ROS_INFO("diff data: %03d %03d %03d %03d",writediff[0],writediff[1],writediff[2],writediff[3]);
  wheelcmd2writediff(wheel_cmd[0],0); //set writediff[0 and 1] from wheel_cmd[0]
  wheelcmd2writediff(wheel_cmd[1],2); //set writediff[2 and 3] from wheel_cmd[0]

  /*writediff[0] = 50;//static_cast<uint8_t>(wheel_cmd[0]);
  writediff[1] = 1;
  writediff[2] = 213;
  writediff[3] = 1;*/

  serialdiff.write(writediff,5);
  ROS_INFO("write diff data: %03d %03d %03d %03d",writediff[0],writediff[1],writediff[2],writediff[3]);
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
