#include "track_line_controller.h"

funcase_controllers::TrackLineController::TrackLineController() :
  error_back(0), initspeed(90.0), k_p(0.4), k_i(0.0), k_d(0.0){

}

funcase_controllers::TrackLineController::~TrackLineController(){

}

bool funcase_controllers::TrackLineController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  // Register node for parent and robot handle
  m_node = node;
  m_robot = robot;


  dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig>::CallbackType f;
  f = boost::bind(&TrackLineController::callback_reconfigure, this, _1, _2);
  m_server.setCallback(f);

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

  turn = (k_p)*static_cast<double>(error) + (k_i)*static_cast<double>(error_sum) + (k_d)*static_cast<double>(error_dot);

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

void funcase_controllers::TrackLineController::callback_reconfigure(funcase_controller::TrackLinePIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  k_p = config.k_p;
  k_i = config.k_i;
  k_d = config.k_d;
  initspeed = config.initspeed;
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineController, controller_interface::ControllerBase)
