#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "controller_manager_msgs/SwitchController.h"

#include "dynamic_reconfigure/Config.h"
#include "dynamic_reconfigure/DoubleParameter.h"

#define SHOW_DEBUG

/****************    Config   *******************/
#define SENSOR_REG_COUNT (6)
#define SIZE_DATA_RECOARD (10)
#define CONVERG_THROSHOLD (M_PI * 5.0/180.0)

#define SWITCH_CONTROLLER_DURATION    (0.2)
#define turndeg_kp (0.8f)

#define TASK_1_SENSOR_THROSHOLD      (100)
#define TASK_3_DEG_THROSHOLD         (90.0f)
#define TASK_4_WAIT_DURATION         (0.1)
//#define TASK_3_ABSDEG_THROSHOLD      (1.0)
#define TASK_6_SENSOR_THROSHOLD      (200)
#define TASK_8_SCAN_FRONT_THROSHOLD  (0.70f)
#define TASK_10_DEG_THROSHOLD        (90.0f)
#define TASK_11_WAIT_DURATION        (0.1)

/************************************************/

/***********************************************/
bool is_sensor_ready(false);
bool is_imu_ready(false);
bool is_laser_ready(false);

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
                       ros::Publisher* _funcase_moveit_pub, ros::Publisher* _dynamic_line_pub, ros::Publisher* _dynamic_wall_pub);
//void changeControllers(int _stage, ros::ServiceClient* _funcase_client, controller_manager_msgs::SwitchController* _switch_control, ros::ServiceClient* _set_IMU_zero, std_srvs::Empty* _emp_srv,
//                       ros::Publisher* _funcase_moveit_pub, std_msgs::Int16MultiArray* _moveit_msg, ros::Publisher* _dynamic_line_pub, ros::Publisher* _dynamic_wall_pub, dynamic_reconfigure::Config* _dynamic_msg);
bool stage_change_detect(int _stage);
double get_sensor_average();
double cot_angle(double _degree);

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
 is_laser_ready = true;
}

/*********************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_main_loop_start");
  ros::NodeHandle node;

  /********************************** Client **********************************/
  ros::ServiceClient funcase_client = node.
      serviceClient<controller_manager_msgs::SwitchController>("/funcasebot/controller_manager/switch_controller");
  ros::ServiceClient IMU_zero_client = node.
      serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");

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
  ros::Publisher dynamic_wall_pub = node.
      advertise<dynamic_reconfigure::Config>("/funcasebot/track_wall_controller/parameter_updates",50);
  ros::Publisher dynamic_line_pub = node.
      advertise<dynamic_reconfigure::Config>("/funcasebot/track_line_controller/parameter_updates",50);


  ros::Rate r(30);
  while (ros::ok())
  {
    if(is_imu_ready){
      ///////////////////////////////////////////////////////
      ///// if stage change do changeControllers() ones /////
      ///////////////////////////////////////////////////////
      if(!is_call && is_laser_ready && is_sensor_ready){
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_pub, &dynamic_wall_pub);
        ROS_INFO("stage %d", stage);
        is_call = true;
      }
      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////

      ///////////////////////////////////////////////////////
      ////////////// check and change stage /////////////////
      ///////////////////////////////////////////////////////
      if(stage == 3){
        //turn deg control
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_pub, &dynamic_wall_pub);
      }else if (stage == 10) {
        //turn deg control
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_pub, &dynamic_wall_pub);
      }

      if(stage == 12){
        //stop at stage 12
      }else if (stage_change_detect(stage)){
        stage++;
        is_call = false;
      }
      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////
    }//end if imu_ready
#ifdef SHOW_DEBUG
    ROS_INFO("yaw: %4.3f",yaw);
#endif
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
/////////////////////////////////// function ///////////////////////////////////

void rotation_Xangle(int _mode, float _angle, std_msgs::Int16MultiArray* _move_msg){
  int16_t turn;
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
                       ros::Publisher* _funcase_moveit_pub, ros::Publisher* _dynamic_line_pub, ros::Publisher* _dynamic_wall_pub){
//void changeControllers(int _stage, ros::ServiceClient* _funcase_client, controller_manager_msgs::SwitchController* _switch_control, ros::ServiceClient* _set_IMU_zero, std_srvs::Empty _emp_srv,
//                      ros::Publisher* _funcase_moveit_pub, std_msgs::Int16MultiArray* _moveit_msg, ros::Publisher* _dynamic_line_pub, ros::Publisher* _dynamic_wall_pub, dynamic_reconfigure::Config* _dynamic_msg){
  controller_manager_msgs::SwitchController switch_control;
  dynamic_reconfigure::Config dynamic_msg;
  std_msgs::Int16MultiArray moveit_msg;
  std_srvs::Empty emp_srv;
  switch_control.request.stop_controllers.clear();
  switch_control.request.start_controllers.clear();
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
    switch_control.request.start_controllers.push_back("track_line_controller");
    SetLineDynamicParams(&dynamic_msg, 0.5,0.0,1.0,150.0);
    switch_enable = true;
    dyline_enable = true;
    break;

  case 1:
    break;

  case 2:
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    break;

  case 3:
    rotation_Xangle(0,90.0,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 4:
    rotation_Xangle(1,90.0,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 5:
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_enable = true;
    break;

  case 6:
    break;

  case 7:
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("track_wall_controller");
    SetWallDynamicParams(&dynamic_msg, 0.5,0.0,1.0,150.0,0.5,1.571);
    switch_enable = true;
    dywall_enable = true;
    break;

  case 8:
    break;

  case 9:
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    break;

  case 10:
    rotation_Xangle(0,90,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 11:
    rotation_Xangle(1,90,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 12:
    //start stack wall
    break;
  }

  if(setzeo_enable)
    _set_IMU_zero->call(emp_srv);
  if(dyline_enable)
    _dynamic_line_pub->publish(dynamic_msg);
  if(dywall_enable)
    _dynamic_wall_pub->publish(dynamic_msg);
  if(switch_enable)
    _funcase_client->call(switch_control);
  if(pubmsg_enable)
    _funcase_moveit_pub->publish(moveit_msg);
}

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
    //sensor almost white
    if(get_sensor_average() < TASK_1_SENSOR_THROSHOLD)
      return true;
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
    break;

  case 4:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_4_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 5:
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

  case 6:
    //sensor all black
    if(get_sensor_average() > TASK_6_SENSOR_THROSHOLD)
      return true;
    break;

  case 7:
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

  case 8:
    //scan front
    if(front_length < TASK_8_SCAN_FRONT_THROSHOLD)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      return true;
    }
    break;

  case 9:
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

  case 10:
    //yaw > 90
    if(yaw > TASK_10_DEG_THROSHOLD)
      return true;
    break;

  case 11:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_11_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 12:
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

double cot_angle(double _degree) {
  if( _degree < 0.0) {
    double times_ = fabs(_degree) / (2*M_PI);
    return ( fmod( _degree + ( 2*M_PI * (floor(times_)+1) ) + M_PI  ,2*M_PI) - M_PI);
  } else if ( _degree > 0.0) {
    return ( fmod( _degree + M_PI ,2*M_PI) - M_PI);
  } else {
    return 0.0;
  }
}
