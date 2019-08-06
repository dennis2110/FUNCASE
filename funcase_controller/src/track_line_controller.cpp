#include "track_line_controller.h"

funcase_controllers::TrackLineController::TrackLineController() :
   error_sum(0), error_back(0), initspeed(90.0), k_p(0.4), k_i(0.0), k_d(0.0){
  is_usetimer = false;
  is_change = false;
}

funcase_controllers::TrackLineController::~TrackLineController(){

}

bool funcase_controllers::TrackLineController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  // Register node for parent and robot handle
  m_node = node;
  m_robot = robot;

  // Initialize dynamic_reconfigure server
  funcase_controller::TrackLinePIDparamConfig config;
  config.k_p = 0.2;
  config.k_i = 0.0;
  config.k_d = 4.5;
  config.initspeed = 100.0;

  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(m_node);
  dyn_reconf_server_->updateConfig(config);
  dyn_reconf_server_->setCallback(boost::bind(&TrackLineController::callback_reconfigure, this, _1, _2));
  //dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig>::CallbackType f;
  //f = boost::bind(&TrackLineController::callback_reconfigure, this, _1, _2);
  //m_server.setCallback(f);

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_sensor_sub = m_node.subscribe<std_msgs::UInt8MultiArray>("/track_line_sensor",1, &TrackLineController::setCommandCB, this);

  return true;
}

void funcase_controllers::TrackLineController::update(const ros::Time &time, const ros::Duration &period){
  //ROS_INFO("controller get sensor : %d %d %d %d %d %d",sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],sensor_data[5]);
  
  
  error = sensor_data[0]*2 + sensor_data[1]*1.5 + sensor_data[2] - sensor_data[3] -sensor_data[4]*1.5 - sensor_data[5]*2;
  
   
  error_sum += error;
  error_dot = error - error_back;
  error_back= error;
  //ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",k_p , k_i, k_d, initspeed);
  //std::cout << "error->" << error << ", error_sum->" << error_sum << ", error_dot->" << error_dot << std::endl;

  turn = (k_p)*static_cast<double>(error) + (k_i)*static_cast<double>(error_sum) + (k_d)*static_cast<double>(error_dot);


  if (k_p > 0){
    if(!is_usetimer){
      if(sensor_data[6] == 1){
        last_time = ros::Time::now();
        is_usetimer = true;
        r_speed = 120;
        l_speed = -150;
        ROS_INFO("turn RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");
      }
      else if(sensor_data[6] == 2){
        last_time = ros::Time::now();
        is_usetimer = true;
        r_speed = -150.0;
        l_speed = 120;
        ROS_INFO("turn LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLl");
      }
    }

    if(is_usetimer){
      m_left_wheel.setCommand(l_speed);
      m_right_wheel.setCommand(r_speed);
      ROS_INFO("turnturnturnturnturnturnturnturnturnturntur");
      if(ros::Time::now().toSec() - last_time.toSec() > 0.2) {
        if((sensor_data[2] < 30) || (sensor_data[3] < 30)){
          is_usetimer = false;
          m_left_wheel.setCommand(0);
          m_right_wheel.setCommand(0);
          ROS_INFO("0000000000000000000000000000000000000000000");
          last_time = ros::Time::now();
        }
      }
    }else{
      m_left_wheel.setCommand(initspeed-turn);
      m_right_wheel.setCommand(initspeed+turn+10.0);
      ROS_INFO("PIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDP");
    }
  }else{
    if(!is_usetimer){
      if(sensor_data[6] == 2){
        last_time = ros::Time::now();
        is_usetimer = true;
        r_speed = 120;
        l_speed = -150;
        ROS_INFO("turn RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");
      }
      else if(sensor_data[6] == 1){
        last_time = ros::Time::now();
        is_usetimer = true;
        r_speed = -150.0;
        l_speed = 120;
        ROS_INFO("turn LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLl");
      }
    }

    if(is_usetimer){
      m_left_wheel.setCommand(l_speed);
      m_right_wheel.setCommand(r_speed);
      ROS_INFO("turnturnturnturnturnturnturnturnturnturntur");
      if(ros::Time::now().toSec() - last_time.toSec() > 0.2) {
        if((sensor_data[2] > 220) || (sensor_data[3] >220)){
          is_usetimer = false;
          m_left_wheel.setCommand(0);
          m_right_wheel.setCommand(0);
          ROS_INFO("0000000000000000000000000000000000000000000");
          last_time = ros::Time::now();
        }
      }
    }else{
      m_left_wheel.setCommand(initspeed-turn);
      m_right_wheel.setCommand(initspeed+turn+10.0);
      ROS_INFO("PIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDPIDP");
    }    
  }
  
  
  
}

void funcase_controllers::TrackLineController::starting(const ros::Time &time){

}

void funcase_controllers::TrackLineController::stopping(const ros::Time &time){
  const double vel = 0.0;
  for (int i = 0; i<2; i++){
    m_left_wheel.setCommand(vel);
    m_right_wheel.setCommand(vel);
  }
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

void funcase_controllers::TrackLineController::setCommand(uint8_t sensor1, uint8_t sensor2, uint8_t sensor3, uint8_t sensor4, uint8_t sensor5, uint8_t sensor6,uint8_t sensor7){
  sensor_data[0] = sensor1;
  sensor_data[1] = sensor2;
  sensor_data[2] = sensor3;
  sensor_data[3] = sensor4;
  sensor_data[4] = sensor5;
  sensor_data[5] = sensor6;
  sensor_data[6] = sensor7;
  //ROS_INFO("sensor %d", sensor_data[3]);
}

void funcase_controllers::TrackLineController::setCommandCB(const std_msgs::UInt8MultiArrayConstPtr &sensor_msg){
  setCommand(sensor_msg->data.at(0),sensor_msg->data.at(1),sensor_msg->data.at(2),sensor_msg->data.at(3),sensor_msg->data.at(4),sensor_msg->data.at(5),sensor_msg->data.at(6));
}

void funcase_controllers::TrackLineController::callback_reconfigure(funcase_controller::TrackLinePIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  k_p = config.k_p;
  k_i = config.k_i;
  k_d = config.k_d;
  initspeed = config.initspeed;
}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineController, controller_interface::ControllerBase)
