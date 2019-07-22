#include "track_line_cv_controller.h"

funcase_controllers::TrackLineCVController::TrackLineCVController() :
   error_sum(0), error_back(0), initspeed(90.0), k_p(0.4), k_i(0.0), k_d(0.0){

}

funcase_controllers::TrackLineCVController::~TrackLineCVController(){

}

bool funcase_controllers::TrackLineCVController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
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
  dyn_reconf_server_->setCallback(boost::bind(&TrackLineCVController::callback_reconfigure, this, _1, _2));
  //dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig>::CallbackType f;
  //f = boost::bind(&TrackLineController::callback_reconfigure, this, _1, _2);
  //m_server.setCallback(f);

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_cv_sub = m_node.subscribe<std_msgs::Float64>("/track_cv_sensor",1, &TrackLineCVController::setCommandCB, this);

  return true;
}

void funcase_controllers::TrackLineCVController::update(const ros::Time &time, const ros::Duration &period){


  error_sum += error;
  error_dot = error - error_back;
  error_back= error;

  turn = (k_p)*static_cast<double>(error) + (k_i)*static_cast<double>(error_sum) + (k_d)*static_cast<double>(error_dot);

  m_left_wheel.setCommand(initspeed-turn);
  m_right_wheel.setCommand(initspeed+turn);
}

void funcase_controllers::TrackLineCVController::starting(const ros::Time &time){

}

void funcase_controllers::TrackLineCVController::stopping(const ros::Time &time){
  const double vel = 0.0;
  for (int i = 0; i<2; i++){
    m_left_wheel.setCommand(vel);
    m_right_wheel.setCommand(vel);
  }
}

bool funcase_controllers::TrackLineCVController::read_parameter(){
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

//void funcase_controllers::TrackLineCVController::setCommand(uint8_t sensor1, uint8_t sensor2, uint8_t sensor3, uint8_t sensor4, uint8_t sensor5, uint8_t sensor6){
//  sensor_data[0] = sensor1;
//  sensor_data[1] = sensor2;
//  sensor_data[2] = sensor3;
//  sensor_data[3] = sensor4;
//  sensor_data[4] = sensor5;
//  sensor_data[5] = sensor6;
//  //ROS_INFO("sensor %d", sensor_data[3]);
//}

void funcase_controllers::TrackLineCVController::setCommandCB(const std_msgs::Float64ConstPtr& cv_error_msg){
  error = cv_error_msg->data;
}

void funcase_controllers::TrackLineCVController::callback_reconfigure(funcase_controller::TrackLinePIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  k_p = config.k_p;
  k_i = config.k_i;
  k_d = config.k_d;
  initspeed = config.initspeed;
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineCVController, controller_interface::ControllerBase)
