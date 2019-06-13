#include "track_wall_controller.h"

funcase_controllers::TrackWallController::TrackWallController() :
  error(0.0), error_sum(0.0), error_back(0.0), initspeed(80.0), k_p(0.1), k_i(0.0), k_d(0.0){

}

funcase_controllers::TrackWallController::~TrackWallController(){

}

bool funcase_controllers::TrackWallController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  m_node = node;
  m_robot = robot;

  // Initialize dynamic_reconfigure server
  funcase_controller::TrackWallPIDparamConfig config;
  config.k_p = 0.4;
  config.k_i = 0.0;
  config.k_d = 0.0;
  config.initspeed = 100.0;
  config.wallrange = 0.3;

  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(m_node);
  dyn_reconf_server_->updateConfig(config);
  dyn_reconf_server_->setCallback(boost::bind(&TrackWallController::callback_reconfigure, this, _1, _2));

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_lidar_sub = m_node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &TrackWallController::setCommandCB, this);

  return true;
}

void funcase_controllers::TrackWallController::update(const ros::Time &time, const ros::Duration &period){
  error = 0.0;
  for (int i=0;i<9;i++) {
    error += lidar_value[i];
  }
  error = error/9 - wallrange;
  ROS_INFO("error = %f",static_cast<double>(error));
  error_sum += error;
  error_dot = error_back - error;
  error_back= error;

  turn = (k_p)*static_cast<double>(error) + (k_i)*static_cast<double>(error_sum) + (k_d)*static_cast<double>(error_dot);

  m_left_wheel.setCommand(initspeed-turn);
  m_right_wheel.setCommand(initspeed+turn);
}

void funcase_controllers::TrackWallController::starting(const ros::Time &time){

}

void funcase_controllers::TrackWallController::stopping(const ros::Time &time){
  const double vel = 0.0;
  for (int i = 0; i<2; i++){
    m_left_wheel.setCommand(vel);
    m_right_wheel.setCommand(vel);
  }
}

bool funcase_controllers::TrackWallController::read_parameter(){
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

void funcase_controllers::TrackWallController::callback_reconfigure(funcase_controller::TrackWallPIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  k_p = config.k_p;
  k_i = config.k_i;
  k_d = config.k_d;
  initspeed = config.initspeed;
  wallrange = static_cast<float>(config.wallrange);
}

void funcase_controllers::TrackWallController::setCommandCB(const sensor_msgs::LaserScanConstPtr &scan_msg){

  for(size_t i = 0;i<9;i++){
    lidar_value[i] = scan_msg->ranges.at(i+535);
  }
  ROS_INFO("wall data: %4.3f %4.3f %4.3f", scan_msg->ranges.at(535), scan_msg->ranges.at(539), scan_msg->ranges.at(543));
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackWallController, controller_interface::ControllerBase)
