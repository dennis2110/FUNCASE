#include "arm_controller.h"

funcase_controllers::ArmController::ArmController(){

}

funcase_controllers::ArmController::~ArmController(){

}

bool funcase_controllers::ArmController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &node){
  m_node = node;
  m_robot = robot;
  
  armcmd_struct.r_arm_pos[0] = 128.0;
  armcmd_struct.r_arm_pos[1] = 128.0;
  armcmd_struct.r_arm_pos[2] = 128.0;
  armcmd_struct.r_arm_pos[3] = 90.0;
  armcmd_struct.r_arm_pos[4] = 180.0;

  armcmd_struct.l_arm_pos[0] = 128.0;
  armcmd_struct.l_arm_pos[1] = 128.0;
  armcmd_struct.l_arm_pos[2] = 128.0;
  armcmd_struct.l_arm_pos[3] = 60.0;

  if(!read_parameter()) return false;

  arm_cmd_sub = m_node.subscribe<std_msgs::Int16MultiArray>("move_arm", 1, &ArmController::setArmCommandCB, this);

  return true;
}

void funcase_controllers::ArmController::update(const ros::Time &time, const ros::Duration &period){
  for (size_t i=0;i<right_arm_joint_num;i++) {
    m_right_arm.at(i).setCommand(static_cast<double>(armcmd_struct.r_arm_pos[i]));
  }
  for (size_t i=0;i<left_arm_joint_num;i++) {
   m_left_arm.at(i).setCommand(static_cast<double>(armcmd_struct.l_arm_pos[i]));
  }
}

void funcase_controllers::ArmController::starting(const ros::Time &time){

}

void funcase_controllers::ArmController::stopping(const ros::Time &time){

}

bool funcase_controllers::ArmController::read_parameter(){
  XmlRpc::XmlRpcValue joint_names;
  if(!m_node.getParam("right_arm",joint_names)){
    ROS_ERROR("No 'right_arm' in controller. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("'right_arm' is not a struct. (namespace: %s)",
                  m_node.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue name_value;
  //r1
  name_value = joint_names[0];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_right_arm.push_back(m_robot->getHandle((std::string)name_value));
  //r2
  name_value = joint_names[1];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_right_arm.push_back(m_robot->getHandle((std::string)name_value));
  //r3
  name_value = joint_names[2];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_right_arm.push_back(m_robot->getHandle((std::string)name_value));
  //r4
  name_value = joint_names[3];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_right_arm.push_back(m_robot->getHandle((std::string)name_value));
  //r5
  name_value = joint_names[4];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_right_arm.push_back(m_robot->getHandle((std::string)name_value));


  //////////////////////// left arm ///////////////////////////////////
  if(!m_node.getParam("left_arm",joint_names)){
    ROS_ERROR("No 'left_arm' in controller. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("'left_arm' is not a struct. (namespace: %s)",
                  m_node.getNamespace().c_str());
    return false;
  }
  //l1
  name_value = joint_names[0];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_left_arm.push_back(m_robot->getHandle((std::string)name_value));
  //l2
  name_value = joint_names[1];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_left_arm.push_back(m_robot->getHandle((std::string)name_value));
  //l3
  name_value = joint_names[2];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_left_arm.push_back(m_robot->getHandle((std::string)name_value));
  //l4
  name_value = joint_names[3];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString){
    ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
    return false;
  }
  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data());
  m_left_arm.push_back(m_robot->getHandle((std::string)name_value));

  return true;
}


void funcase_controllers::ArmController::setArmCommandCB(const std_msgs::Int16MultiArrayConstPtr &arm_msg){
  const int16_t* ptr = arm_msg->data.data();
  for (int i=0;i<right_arm_joint_num;i++) {
    armcmd_struct.r_arm_pos[i] = static_cast<double>(ptr[i]);
  }
  for(int i=0;i<left_arm_joint_num;i++){
    armcmd_struct.l_arm_pos[i] = static_cast<double>(ptr[i+5]);
  }
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::ArmController, controller_interface::ControllerBase)
