#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "controller_manager_msgs/SwitchController.h"

#include "dynamic_reconfigure/Reconfigure.h"
#include "dynamic_reconfigure/Config.h"
#include "dynamic_reconfigure/DoubleParameter.h"

#define SHOW_DEBUG

/****************    Config   *******************/
#define SENSOR_REG_COUNT (6)
#define SIZE_DATA_RECOARD (10)
#define CONVERG_THROSHOLD (M_PI * 5.0/180.0)

#define SWITCH_CONTROLLER_DURATION    (0.2)
#define turndeg_kp (0.8f)
#define ORIENT_RIGHT_KP  (5.0f)
#define ORIENT_RIGHT_KD  (5000.0f)

#define TASK_1_WAIT_DURATION         (2.0)
#define TASK_3_WAIT_DURATION         (0.1)

/************************************************/

/***********************************************/
int16_t turn;
float error;
float error_dot = 0.0;
float error_back = 0.0;
float wallrange = 0.0;
/***********************************************/
bool is_sensor_ready(false);
bool is_imu_ready(false);
bool is_laser_ready(false);

float laser_angle_max;
float laser_angle_min;
float laser_angle_increment;
float laser_count_max;

uint8_t sensor_value[SENSOR_REG_COUNT] = { 0 };
float yaw(0.0);
float front_length(10.0);
float right_length(10.0);
float left_length(10.0);

int stage(0);
bool is_call(false);

sensor_msgs::LaserScan laser_msg;

void rotation_Xangle(int _mode, float _angle, std_msgs::Int16MultiArray* _move_msg);
void SetLineDynamicParams(dynamic_reconfigure::Config* _dynamic_msg, double _k_p, double _k_i, double _k_d, double _initspeed);
void SetWallDynamicParams(dynamic_reconfigure::Config* _dynamic_msg, double _k_p, double _k_i, double _k_d, double _initspeed, double _wallrange, double _wallangle);
void changeControllers(int _stage, ros::ServiceClient* _funcase_client,ros::ServiceClient* _set_IMU_zero,
                       ros::Publisher* _funcase_moveit_pub, ros::ServiceClient* _dynamic_line_client, ros::ServiceClient* _dynamic_wall_client);
bool stage_change_detect(int _stage);

double get_sensor_average();
float cot_angle(float _degree);
inline float get_laser_distence(float _angle);
inline float get_right_distence(float _orient_offset);
inline float get_left_distence(float _orient_offset);
inline float get_front_distence(float _orient_offset);


/***********************************************/

/******************* callback ****************************/
void callback_sensor(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  const uint8_t* ptr = msg->data.data();
  for (int i = 0; i < 5; i++) {
    sensor_value[i] = ptr[i];
  }
  is_sensor_ready = true;
}
void callback_IMU_yaw(const std_msgs::Float32ConstPtr& msg){
  yaw = msg->data;
  is_imu_ready = true;
}
void callback_scan(const sensor_msgs::LaserScan msg){
  // Initial the Laser data at first time received /scan topic
  if( !is_laser_ready ) {
    laser_angle_max = msg.angle_max;
    laser_angle_min = msg.angle_min;
    laser_angle_increment = msg.angle_increment;
    laser_count_max = (laser_angle_max - laser_angle_min) / laser_angle_increment - 1;

    is_laser_ready = true;
  }

  laser_msg = msg;

  float length_sum(0.0);
  for(size_t i=0;i<9;i++)
  length_sum += msg.ranges.at(i+81);
  right_length = length_sum / 9;

  length_sum = 0.0;
  for(size_t i=0;i<9;i++)
    length_sum += msg.ranges.at(i+337);
  front_length = length_sum / 9;

  length_sum = 0.0;
  for(size_t i=0;i<9;i++)
    length_sum += msg.ranges.at(i+593);
  left_length = length_sum / 9;
}

/*********************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_slope");
  ros::NodeHandle node;

  /********************************** Client **********************************/
  ros::ServiceClient funcase_client = node.
      serviceClient<controller_manager_msgs::SwitchController>("/funcasebot/controller_manager/switch_controller");
  ros::ServiceClient IMU_zero_client = node.
      serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");
  ros::ServiceClient dynamic_line_client = node.
      serviceClient<dynamic_reconfigure::Reconfigure>("/funcasebot/track_line_controller/set_parameters");
  ros::ServiceClient dynamic_wall_client = node.
      serviceClient<dynamic_reconfigure::Reconfigure>("/funcasebot/track_wall_controller/set_parameters");

  /******************************** Subscriber ********************************/
  ros::Subscriber cny70_sub = node.
      subscribe<std_msgs::UInt8MultiArray>("/track_line_sensor", 50, callback_sensor);
  ros::Subscriber IMU_yaw_sub = node.
      subscribe<std_msgs::Float32>("/imu/yaw", 50, callback_IMU_yaw);
  ros::Subscriber scan_sub = node.
      subscribe<sensor_msgs::LaserScan>("/scan", 50, callback_scan);

  /******************************** Publishers ********************************/
  ros::Publisher move_it_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",50);


  ros::Rate r(30);
  while (ros::ok())
  {
    //if(is_sensor_ready && is_laser_ready){
    if(is_laser_ready  && is_imu_ready && is_sensor_ready){
    ///////////////////////////////////////////////////////
    ///// if stage change do changeControllers() ones /////
    ///////////////////////////////////////////////////////
      if(!is_call && is_laser_ready && is_sensor_ready){
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
        ROS_INFO("stage %d", stage);
        is_call = true;
      }
    ///////////////////////////////////////////////////////
    ///// if stage change do changeControllers() ones /////
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    ///////////// repeat changeControllers() //////////////
    ///////////////////////////////////////////////////////
      if(stage == 6){
        //turn deg control
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 13) {
        //turn deg control
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 4){
        //fuzzy decelerate
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }
    ///////////////////////////////////////////////////////
    ///////////// repeat changeControllers() //////////////
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    ////////////// check and change stage /////////////////
    ///////////////////////////////////////////////////////
      if(stage == 3){
        //stop at stage 3
      }else if(stage_change_detect(stage)){
        stage++;
        is_call = false;
      }
    ///////////////////////////////////////////////////////
    ////////////// check and change stage /////////////////
    ///////////////////////////////////////////////////////
    }//end if imu_ready
#ifdef SHOW_DEBUG
    ROS_INFO("yaw: %4.3f",yaw);
    ROS_INFO("stage: %d",stage);
#endif
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// function ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void rotation_Xangle(int _mode, float _angle, std_msgs::Int16MultiArray* _move_msg){
  switch (_mode) {
  case 0:
    turn = static_cast<int16_t>(turndeg_kp * (_angle - yaw));
    if(turn > 0){
      _move_msg->data.push_back(-turn-80);
      _move_msg->data.push_back(turn+80);
    }else{
      _move_msg->data.push_back(-turn+80);
      _move_msg->data.push_back(turn-80);
    }
    break;
  case 1:
    _move_msg->data.push_back(0);
    _move_msg->data.push_back(0);
    break;

  }
}
void SetLineDynamicParams(dynamic_reconfigure::Config* _dynamic_msg, double _k_p, double _k_i, double _k_d, double _initspeed){
  dynamic_reconfigure::DoubleParameter dynamic_double;

  dynamic_double.name = "k_p";
  dynamic_double.value = _k_p;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "k_i";
  dynamic_double.value = _k_i;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "k_d";
  dynamic_double.value = _k_d;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "initspeed";
  dynamic_double.value = _initspeed;
  _dynamic_msg->doubles.push_back(dynamic_double);
}
void SetWallDynamicParams(dynamic_reconfigure::Config* _dynamic_msg, double _k_p, double _k_i, double _k_d, double _initspeed, double _wallrange, double _wallangle){
  dynamic_reconfigure::DoubleParameter dynamic_double;

  dynamic_double.name = "k_p";
  dynamic_double.value = _k_p;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "k_i";
  dynamic_double.value = _k_i;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "k_d";
  dynamic_double.value = _k_d;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "initspeed";
  dynamic_double.value = _initspeed;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "wallrange";
  dynamic_double.value = _wallrange;
  _dynamic_msg->doubles.push_back(dynamic_double);
  dynamic_double.name = "wallangle";
  dynamic_double.value = _wallangle;
  _dynamic_msg->doubles.push_back(dynamic_double);
}
void changeControllers(int _stage, ros::ServiceClient* _funcase_client,ros::ServiceClient* _set_IMU_zero,
                       ros::Publisher* _funcase_moveit_pub, ros::ServiceClient* _dynamic_line_client, ros::ServiceClient* _dynamic_wall_client){
  controller_manager_msgs::SwitchController switch_control;
  dynamic_reconfigure::Reconfigure dynamic_srv;
  dynamic_reconfigure::Config dynamic_msg;
  std_msgs::Int16MultiArray moveit_msg;
  std_srvs::Empty emp_srv;
  switch_control.request.stop_controllers.clear();
  switch_control.request.start_controllers.clear();
  dynamic_srv.request.config.doubles.clear();
  moveit_msg.data.clear();
  dynamic_msg.doubles.clear();
  /*** change stage ****/
  bool setzeo_enable(false);
  bool switch_enable(false);
  bool pubmsg_enable(false);
  bool dyline_enable(false);
  bool dywall_enable(false);
  switch (_stage) {
  case 0:
    //start track wall, speed = 170, slope low
    wallrange = right_length;
    switch_control.request.start_controllers.push_back("track_wall_controller");
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,170.0,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    dywall_enable = true;
    switch_enable = true;
    setzeo_enable = true;
    break;

  case 1:
    //slow down speed, speed = 100
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,100.0,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    dywall_enable = true;
    break;

  case 2:
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;

  case 3:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;
  }

  if(setzeo_enable)
    _set_IMU_zero->call(emp_srv);
  if(dyline_enable)
    _dynamic_line_client->call(dynamic_srv);
  if(dywall_enable)
    _dynamic_wall_client->call(dynamic_srv);
  if(switch_enable)
    _funcase_client->call(switch_control);
  if(pubmsg_enable)
    _funcase_moveit_pub->publish(moveit_msg);
}
//  bool stage_change_detect(int _stage);

bool stage_change_detect(int _stage){
  // Count the times of robot distence over minimum limit
  static int laser_distence_overlimit_conter(0);
  static bool fg_usetimer(false);
  static ros::Time last_time = ros::Time::now();

  switch (_stage) {
  case 0:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > SWITCH_CONTROLLER_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 1:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_1_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 2:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > SWITCH_CONTROLLER_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 3:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_3_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;
  }

  // Default False
  return false;
}

double get_sensor_average() {
    int sum(0);
    for(int i=0; i<SENSOR_REG_COUNT; i++) {
        sum += sensor_value[i];
    }

    return sum / SENSOR_REG_COUNT;
}

float cot_angle(float _degree) {
  if( _degree < 0.0f) {
    float times_ = fabs(_degree) / (2*M_PIf32);
    return ( fmod( _degree + ( 2*M_PIf32 * (floor(times_)+1) ) + M_PIf32  ,2*M_PIf32) - M_PIf32);
  } else if ( _degree > 0.0f) {
    return ( fmod( _degree + M_PIf32 ,2*M_PIf32) - M_PIf32);
  } else {
    return 0.0;
  }
}

float get_laser_distence(float _angle) {

    float _ref_orient_increment_diff = _angle - laser_angle_min;
    float _ref_count = _ref_orient_increment_diff / laser_angle_increment;

    if( _ref_count < 0)
        _ref_count = 1;
    else if( _ref_count > laser_count_max)
        _ref_count = laser_count_max;
    ///add 7/29
    printf("count: %d\n", static_cast<int>(_ref_count));
    printf("ranges: %4.3f\n", laser_msg.ranges.at(static_cast<size_t>(_ref_count)));
    ///add 7/29

    return laser_msg.ranges.at(static_cast<size_t>(_ref_count));
}

float get_right_distence(float _orient_offset) { return get_laser_distence( -M_PIf32/2 - _orient_offset); }

float get_left_distence(float _orient_offset) { return get_laser_distence( M_PIf32/2 - _orient_offset); }

float get_front_distence(float _orient_offset) { return get_laser_distence( _orient_offset ); }

//2019 07 30
