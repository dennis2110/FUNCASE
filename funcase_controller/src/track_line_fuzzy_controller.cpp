#include "track_line_fuzzy_controller.h"

funcase_controllers::TrackLineFuzzyController::TrackLineFuzzyController() :
   initspeed(90.0){

}

funcase_controllers::TrackLineFuzzyController::~TrackLineFuzzyController(){

}

bool funcase_controllers::TrackLineFuzzyController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  // Register node for parent and robot handle
  m_node = node;
  m_robot = robot;

  // Initialize dynamic_reconfigure server
  funcase_controller::TrackLinePIDparamConfig config;
  config.k_p = 0.4;
  config.k_i = 0.0;
  config.k_d = 0.0;
  config.initspeed = 100.0;

  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(m_node);
  dyn_reconf_server_->updateConfig(config);
  dyn_reconf_server_->setCallback(boost::bind(&TrackLineFuzzyController::callback_reconfigure, this, _1, _2));
  //dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig>::CallbackType f;
  //f = boost::bind(&TrackLineController::callback_reconfigure, this, _1, _2);
  //m_server.setCallback(f);

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_sensor_sub = m_node.subscribe<std_msgs::UInt8MultiArray>("/track_line_sensor",1, &TrackLineFuzzyController::setCommandCB, this);

  return true;
}

void funcase_controllers::TrackLineFuzzyController::update(const ros::Time &time, const ros::Duration &period){
  ROS_INFO("controller get sensor : %d %d %d %d %d",sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4]);
  //////////////////////////////////////////////////////
  linefuzzy.nowstatus(sensor_data[0], sensor_data[1], sensor_data[2],
      sensor_data[3], sensor_data[4], sensor_data[5]);
  linefuzzy.fuzzify();
  turn = static_cast<double>(linefuzzy.defuzzify());
  //////////////////////////////////////////////////////

  m_left_wheel.setCommand(initspeed-turn);
  m_right_wheel.setCommand(initspeed+turn);
}

void funcase_controllers::TrackLineFuzzyController::starting(const ros::Time &time){

}

void funcase_controllers::TrackLineFuzzyController::stopping(const ros::Time &time){
  const double vel = 0.0;
  for (int i = 0; i<2; i++){
    m_left_wheel.setCommand(vel);
    m_right_wheel.setCommand(vel);
  }
}

bool funcase_controllers::TrackLineFuzzyController::read_parameter(){
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

void funcase_controllers::TrackLineFuzzyController::setCommand(uint8_t sensor1, uint8_t sensor2, uint8_t sensor3, uint8_t sensor4, uint8_t sensor5, uint8_t sensor6){
  sensor_data[0] = sensor1;
  sensor_data[1] = sensor2;
  sensor_data[2] = sensor3;
  sensor_data[3] = sensor4;
  sensor_data[4] = sensor5;
  sensor_data[5] = sensor6;
  //ROS_INFO("sensor %d", sensor_data[3]);
}

void funcase_controllers::TrackLineFuzzyController::setCommandCB(const std_msgs::UInt8MultiArrayConstPtr &sensor_msg){
  setCommand(sensor_msg->data.at(0),sensor_msg->data.at(1),sensor_msg->data.at(2),sensor_msg->data.at(3),sensor_msg->data.at(4),sensor_msg->data.at(5));
}

void funcase_controllers::TrackLineFuzzyController::callback_reconfigure(funcase_controller::TrackLinePIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  initspeed = config.initspeed;
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineFuzzyController, controller_interface::ControllerBase)
