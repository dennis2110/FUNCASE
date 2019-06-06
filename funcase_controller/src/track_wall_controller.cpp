#include "track_wall_controller.h"

funcase_controllers::TrackWallController::TrackWallController() :
  error_back(0), initspeed(80.0), k_p(0.1), k_i(0.0), k_d(0.0){

}

funcase_controllers::TrackWallController::~TrackWallController(){

}

bool funcase_controllers::TrackWallController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){

}

void funcase_controllers::TrackWallController::update(const ros::Time &time, const ros::Duration &period){

}

void funcase_controllers::TrackWallController::starting(const ros::Time &time){

}

void funcase_controllers::TrackWallController::stopping(const ros::Time &time){

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

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackWallController, controller_interface::ControllerBase)
