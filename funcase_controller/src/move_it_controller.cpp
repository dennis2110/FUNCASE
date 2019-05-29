#include "move_it_controller.h"

funcase_controllers::MoveItController::MoveItController(){

}

funcase_controllers::MoveItController::~MoveItController(){

}

bool funcase_controllers::MoveItController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  m_node = node;
  m_robot = robot;
  movecmd_struct.l_wheel_vel = 0;
  movecmd_struct.r_wheel_vel = 0;

  if(!read_parameter()) return false;

  move_cmd_sub = m_node.subscribe<std_msgs::Int16MultiArray>("move_it",1, &MoveItController::setMoveCommandCB, this);

  return true;
}

void funcase_controllers::MoveItController::update(const ros::Time &time, const ros::Duration &period){
  m_left_wheel.setCommand(static_cast<double>(movecmd_struct.l_wheel_vel));
  m_right_wheel.setCommand(static_cast<double>(movecmd_struct.r_wheel_vel));
}

void funcase_controllers::MoveItController::starting(const ros::Time &time){

}

void funcase_controllers::MoveItController::stopping(const ros::Time &time){

}

bool funcase_controllers::MoveItController::read_parameter(){
  XmlRpc::XmlRpcValue joint_names;
  if(!m_node.getParam("wheels",joint_names)){
    ROS_ERROR("No 'wheel joints' in controller. (namespace: %s)",
                    m_node.getNamespace().c_str());
    return false;
  }
  if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("'wheel joints' is not a struct. (namespace: %s)",
                  m_node.getNamespace().c_str());
    return false;
  }
  XmlRpc::XmlRpcValue name_value;
  name_value = joint_names[0];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
      ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
      return false;
  }

  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
  m_left_wheel=m_robot->
          getHandle((std::string)name_value);
  //joint  l state
  //m_l_turtle_state = m_turtle_state->
  //        getHandle((std::string)name_value);
  //end
  name_value = joint_names[1];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
      ROS_ERROR("wheel joints are not strings. (namespace: %s)",
             m_node.getNamespace().c_str());
      return false;
  }

  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
  m_right_wheel=m_robot->
          getHandle((std::string)name_value);

  return true;
}

void funcase_controllers::MoveItController::setMoveCommand(int16_t left_vel, int16_t right_vel){
  movecmd_struct.l_wheel_vel = left_vel;
  movecmd_struct.r_wheel_vel = right_vel;
}

void funcase_controllers::MoveItController::setMoveCommandCB(const std_msgs::Int16MultiArrayConstPtr &move_msg){
  setMoveCommand(move_msg->data.at(0), move_msg->data.at(1));
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::MoveItController, controller_interface::ControllerBase)
