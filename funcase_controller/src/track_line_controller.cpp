#include "track_line_controller.h"

funcase_controllers::TrackLineController::TrackLineController(){

}

funcase_controllers::TrackLineController::~TrackLineController(){

}

bool funcase_controllers::TrackLineController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  // Register node for parent and robot handle
  m_node = node;
  m_robot = robot;

  //init PID controller
  initspeed = 90;
  error_back = 0;
  Kp = 0.4;
  Ki = 0;
  Kd = 0;

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_sensor_sub = m_node.subscribe<std_msgs::UInt8MultiArray>("/track_line_sensor",1, &TrackLineController::setCommandCB, this);

  return true;
}

void funcase_controllers::TrackLineController::update(const ros::Time &time, const ros::Duration &period){
  ROS_INFO("controller get sensor : %d %d %d %d",sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3]);
  error = sensor_data[0] * 2 + sensor_data[1] - sensor_data[2] - sensor_data[3] * 2;
  error_sum += error;
  error_dot = error_back - error;
  error_back= error;

  turn = Kp*static_cast<double>(error);

  m_left_wheel.setCommand(initspeed-turn);
  m_right_wheel.setCommand(initspeed+turn);
}

void funcase_controllers::TrackLineController::starting(const ros::Time &time){

}

void funcase_controllers::TrackLineController::stopping(const ros::Time &time){

}

bool funcase_controllers::TrackLineController::read_parameter(){
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

void funcase_controllers::TrackLineController::setCommand(uint8_t sensor1, uint8_t sensor2, uint8_t sensor3, uint8_t sensor4){
  sensor_data[0] = sensor1;
  sensor_data[1] = sensor2;
  sensor_data[2] = sensor3;
  sensor_data[3] = sensor4;
}

void funcase_controllers::TrackLineController::setCommandCB(const std_msgs::UInt8MultiArrayConstPtr &sensor_msg){
  setCommand(sensor_msg->data.at(0),sensor_msg->data.at(1),sensor_msg->data.at(2),sensor_msg->data.at(3));
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineController, controller_interface::ControllerBase)
