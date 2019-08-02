#include "funcase_hw.h"

FuncaseRobot::FuncaseRobot() : /*serialimu("/dev/mpu6050",5),*/ serialdiff("/dev/chassis",5), serialarm("/dev/ttyTHS1",5){
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
  for (int i=0;i<5;i++){
    r_arm_eff[i] = 0;
    r_arm_pos[i] = 0;
    r_arm_vel[i] = 0;
  }
  for (int i=0;i<4;i++){
    l_arm_eff[i] = 0;
    l_arm_pos[i] = 0;
    l_arm_vel[i] = 0;
  }
  r_arm_cmd[0] = 128;
  r_arm_cmd[1] = 128;
  r_arm_cmd[2] = 128;
  r_arm_cmd[3] = 150;
  r_arm_cmd[4] = 180;

  l_arm_cmd[0] = 128;
  l_arm_cmd[1] = 128;
  l_arm_cmd[2] = 128;
  l_arm_cmd[3] = 70;

  // connect and register the joint state interface
  //wheel
  hardware_interface::JointStateHandle state_handle_a(
        "base_left_wheel_joint", &wheel_pos[0], &wheel_vel[0], &wheel_eff[0]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b(
        "base_right_wheel_joint", &wheel_pos[1], &wheel_vel[1], &wheel_eff[1]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_b);

  //arm right
  hardware_interface::JointStateHandle state_handle_right_arm_base(
        "right_arm_base_link_joint", &r_arm_pos[0], &r_arm_vel[0], &r_arm_eff[0]);
  jnt_state_interface.registerHandle(state_handle_right_arm_base);

  hardware_interface::JointStateHandle state_handle_right_arm_2(
        "right_arm_2_link_joint", &r_arm_pos[1], &r_arm_vel[1], &r_arm_eff[1]);
  jnt_state_interface.registerHandle(state_handle_right_arm_2);

  hardware_interface::JointStateHandle state_handle_right_arm_3(
        "right_arm_3_link_joint", &r_arm_pos[2], &r_arm_vel[2], &r_arm_eff[2]);
  jnt_state_interface.registerHandle(state_handle_right_arm_3);

  hardware_interface::JointStateHandle state_handle_right_arm_4(
        "right_arm_4_link_joint", &r_arm_pos[3], &r_arm_vel[3], &r_arm_eff[3]);
  jnt_state_interface.registerHandle(state_handle_right_arm_4);

  hardware_interface::JointStateHandle state_handle_right_arm_5(
        "right_arm_5_link_joint", &r_arm_pos[4], &r_arm_vel[4], &r_arm_eff[4]);
  jnt_state_interface.registerHandle(state_handle_right_arm_5);

  //arm left
  hardware_interface::JointStateHandle state_handle_left_arm_base(
        "left_arm_base_link_joint", &l_arm_pos[0], &l_arm_vel[0], &l_arm_eff[0]);
  jnt_state_interface.registerHandle(state_handle_left_arm_base);

  hardware_interface::JointStateHandle state_handle_left_arm_2(
        "left_arm_2_link_joint", &l_arm_pos[1], &l_arm_vel[1], &l_arm_eff[1]);
  jnt_state_interface.registerHandle(state_handle_left_arm_2);

  hardware_interface::JointStateHandle state_handle_left_arm_3(
        "left_arm_3_link_joint", &l_arm_pos[2], &l_arm_vel[2], &l_arm_eff[2]);
  jnt_state_interface.registerHandle(state_handle_left_arm_3);

  hardware_interface::JointStateHandle state_handle_left_arm_4(
        "left_arm_4_link_joint", &l_arm_pos[3], &l_arm_vel[3], &l_arm_eff[3]);
  jnt_state_interface.registerHandle(state_handle_left_arm_4);

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
        jnt_state_interface.getHandle("right_arm_base_link_joint"), &r_arm_cmd[0]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_base);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_2(
        jnt_state_interface.getHandle("right_arm_2_link_joint"), &r_arm_cmd[1]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_2);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_3(
        jnt_state_interface.getHandle("right_arm_3_link_joint"), &r_arm_cmd[2]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_3);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_4(
        jnt_state_interface.getHandle("right_arm_4_link_joint"), &r_arm_cmd[3]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_4);
  hardware_interface::JointHandle pos_cmd_handle_right_arm_5(
        jnt_state_interface.getHandle("right_arm_5_link_joint"), &r_arm_cmd[4]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_right_arm_5);
  //arm left
  hardware_interface::JointHandle pos_cmd_handle_left_arm_base(
        jnt_state_interface.getHandle("left_arm_base_link_joint"), &l_arm_cmd[0]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_left_arm_base);
  hardware_interface::JointHandle pos_cmd_handle_left_arm_2(
        jnt_state_interface.getHandle("left_arm_2_link_joint"), &l_arm_cmd[1]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_left_arm_2);
  hardware_interface::JointHandle pos_cmd_handle_left_arm_3(
        jnt_state_interface.getHandle("left_arm_3_link_joint"), &l_arm_cmd[2]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_left_arm_3);
  hardware_interface::JointHandle pos_cmd_handle_left_arm_4(
        jnt_state_interface.getHandle("left_arm_4_link_joint"), &l_arm_cmd[3]);
  jnt_pos_interface.registerHandle(pos_cmd_handle_left_arm_4);

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
  serialdiff.read();
  //serialarm.read();
  //readarm[0] = serialarm.raw_arm[0];

  for(int i=0;i<SENSOR_REG_COUNT;i++){
    cny70[i] = serialdiff.raw_diff[i];
  }

  //ROS_INFO("read cny70 data: %03d %03d %03d %03d",cny70[0],cny70[1],cny70[2],cny70[3]);
  //ROS_INFO("read imu data: %4.3f %4.3f %4.3f %4.3f",orientation[0],orientation[1],orientation[2],orientation[3]);
  //ROS_INFO("read arm data: %0.d",readarm[0]);
  //publish_sensor_data();
}

void FuncaseRobot::write(){
  wheelcmd2writediff(wheel_cmd[0],0); //set writediff[0 and 1] from wheel_cmd[0]
  wheelcmd2writediff(wheel_cmd[1],2); //set writediff[2 and 3] from wheel_cmd[0]
  armcmd2writearm(r_arm_cmd, l_arm_cmd);

  serialdiff.write(writediff,5);
  serialarm.write(writearm, 17);


  ROS_INFO("diff data: %03d %03d %03d %03d",writediff[0],writediff[1],writediff[2],writediff[3]);
  ROS_INFO("write right arm data: %03d %03d %03d %03d %03d",writearm[0],writearm[2],writearm[4],writearm[6],writearm[8]);
  ROS_INFO("write left arm data: %03d %03d %03d %03d",writearm[10],writearm[12],writearm[14],writearm[16]);
}

void FuncaseRobot::wheelcmd2writediff(double cmd,int n){
  uint8_t temp;
  if(cmd == 0.0){
    writediff[n] = 255;
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

void FuncaseRobot::armcmd2writearm(double* r_cmd, double* l_cmd){
  for (int i=1;i<16;i+=2) {
    writearm[i]=255;
  }
  for(int i=0;i<5;i++){
    writearm[i*2] = static_cast<uint8_t>(r_cmd[i]);
  }
  for(int i=0;i<4;i++){
    writearm[i*2+10] = static_cast<uint8_t>(l_cmd[i]);
  }
}

void FuncaseRobot::publish_sensor_data(){
  std_msgs::UInt8MultiArray sensor_msg;
#ifdef NORMALIZE_CNY70
  sensor_msg.data.push_back(normalize(cny70[0], 110, 10));
  sensor_msg.data.push_back(normalize(cny70[1], 110, 10));
  sensor_msg.data.push_back(normalize(cny70[2], 110, 10));
  sensor_msg.data.push_back(normalize(cny70[3], 90, 10));
  sensor_msg.data.push_back(normalize(cny70[4], 110, 10));
  sensor_msg.data.push_back(normalize(cny70[5], 110, 10));
#else
  for (int i=0;i<SENSOR_REG_COUNT;i++) {
    sensor_msg.data.push_back(cny70[i]);
  }
#endif
  m_track_line_pub.publish(sensor_msg);
}

uint8_t FuncaseRobot::normalize(uint8_t value, uint8_t max, uint8_t min){
  float num = static_cast<float>(value - min )/static_cast<float>(max -  min)*255;
  if(num > 255)
    num = 255.0;
  else if(num < 0)
    num = 0.0;
  return static_cast<uint8_t>(num);
}
