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
#define DECELERATION_KP  (40)
#define ORIENT_RIGHT_KP  (5.0f)
#define ORIENT_RIGHT_KD  (5000.0f)

#define TASK_1_WAIT_DURATION         (1.2)
#define TASK_2_WAIT_DURATION         (0.9)
#define TASK_3_WAIT_DURATION         (0.5)
#define TASK_8_WAIT_DURATION         (0.1)
#define TASK_15_WAIT_DURATION        (0.1)
#define TASK_16_WAIT_DURATION        (0.1)
#define TASK_17_WAIT_DURATION        (0.1)

#define TASK_5_SENSOR_THROSHOLD      (90)
#define TASK_10_SENSOR_THROSHOLD     (200)

#define TASK_7_DEG_THROSHOLD         (90.0f)
#define TASK_14_DEG_THROSHOLD        (90.0f)

#define TASK_12_SCAN_FRONT_THROSHOLD  (0.70f)
/************************************************/

/***********************************************/
int16_t turn;
float error;
float error_dot = 0.0;
float error_back = 0.0;
float wallrange = 0.0;

double speed;
double back_speed = 0.0;
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

int stage(4);
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
class ConvergDetector {
public:
    ConvergDetector() :
        index_counter(0),
        fg_inited(false),
        fg_started(false),
        ref_data(0.0) {}

    ~ConvergDetector() {}

private:

    // Flag for Detector status
    bool fg_inited;
    bool fg_started;
    int index_counter;

    // Detector registers
    double ref_data;
    double data_recoard[SIZE_DATA_RECOARD];
public:
    // Need init first
    void init(double _ref) {
        fg_inited = true;
        ref_data = _ref;
        for(int i=0;i<SIZE_DATA_RECOARD;i++) {
            data_recoard[i] = 10.0;
        }
    }

    bool start() {
        if(fg_inited) {
            fg_started = true;
            return true;
        }
        else {
            return false;
        }
    }

    void stop() {
        fg_started = false;
        fg_inited = false;
    }

    void update() {
        if(fg_started) {
            data_recoard[index_counter] = cot_angle(fabs(ref_data - yaw));
            index_counter++;
            if(index_counter == SIZE_DATA_RECOARD) index_counter = 0;
            ROS_INFO_NAMED("ConvergDetector", "Debug : Error= %f ,ref_data = %f,orient = %f",
                           fabs(ref_data - yaw),ref_data,yaw);
        }
        else {
            ROS_ERROR_NAMED("ConvergDetector", "Need start first!");
        }
    }

    bool isConverged() {

        if(fg_started) {
            double sum(0);
            double avg(0);
            for(int i=0;i<SIZE_DATA_RECOARD;i++) {
                sum += data_recoard[i];
            }
            avg = sum / SIZE_DATA_RECOARD;
            ROS_INFO_NAMED("ConvergDetector", "Debug : Avg = %f ",avg);
            return (CONVERG_THROSHOLD > avg);

        }

        else {
            ROS_ERROR_NAMED("ConvergDetector", "Need start first!");
            return false;
        }

    }

    bool isStarted() {
        return fg_started;
    }
};

class WBDetector {
public:
    WBDetector() :
        fg_inited(false),
        fg_started(false) {}

    ~WBDetector() {}

private:

    // Flag for Detector status
    bool fg_inited;
    bool fg_started;
    bool fg_w2b;

    int head;
    int w_detected;
    int b_detected;

public:
    // Need init first
    void init(bool _is_w2b) {
        fg_w2b = _is_w2b;
        head = 0;
        w_detected = -1;
        b_detected = -1;
        fg_inited = true;
    }

    bool start() {
        if(fg_inited) {
            fg_started = true;
            return true;
        }
        else {
            return false;
        }
    }

    void stop() {
        fg_started = false;
        fg_inited = false;
    }

    void update() {
        ROS_DEBUG_NAMED("WBDetector", "w_detected:%d, b_detected:%d, sensor AVG: %f",
                        w_detected, b_detected, get_sensor_average());
        if(fg_started) {
            if( get_sensor_average() > 70.0 && b_detected < 0)
                b_detected = head ++;
            if( get_sensor_average() < 30.0 && w_detected < 0)
                w_detected = head ++;
        }
        else {
            ROS_ERROR_NAMED("WBDetector", "Need start first!");
        }
    }

    bool isDetected() {
        if(fg_started) {
            if(b_detected < 0 || w_detected < 0) {
                return false;
            }
            else {
                if(fg_w2b)
                    return b_detected > w_detected;
                else
                    return w_detected > b_detected;
            }
        }
        else {
            ROS_ERROR_NAMED("WBDetector", "Need start first!");
            return false;
        }

    }

    bool isStarted() {
        return fg_started;
    }
};

ConvergDetector stable_detector;
WBDetector wb_detector;
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
      if(stage == 301){
        //stop at stage 3
      }else if(stage == 666){
          if(stage_change_detect(stage)){
            stage = 301;
            is_call = false;
          }
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
      _move_msg->data.push_back(turn+80);
      _move_msg->data.push_back(-turn-80);
    }else{
      _move_msg->data.push_back(turn-80);
      _move_msg->data.push_back(-turn+80);
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
    //start moveit controller, set IMU zero
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    break;

  case 1:
    //move farward
    moveit_msg.data.push_back(100);
    moveit_msg.data.push_back(130);
    pubmsg_enable = true;
    break;

  case 2:
    //turn right
    moveit_msg.data.push_back(-100);
    moveit_msg.data.push_back(150);
    pubmsg_enable = true;
    break;

  case 3:
    //move farward
    moveit_msg.data.push_back(140);
    moveit_msg.data.push_back(140);
    pubmsg_enable = true;
    break;

  case 301:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;

  case 4:
    //start trackline controller, set trackline params
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,130.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    switch_enable = true;
    //
    setzeo_enable = true;
    //
    break;

  case 5:
    //dynamic deceleration (line)
    if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * DECELERATION_KP);
    }
    if(speed > 130.0)
      speed = 130.0;
    if(speed < 70.0)
      speed = 70.0;
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;
    break;

  case 6:
    //start moveit controller
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    break;

  case 7:
    //turn left
    moveit_msg.data.push_back(120);
    moveit_msg.data.push_back(-120);
    //rotation_Xangle(0,90.0,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 8:
    //at M_PI * 0/180 stop
    rotation_Xangle(1,90.0,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 9:
    //start trackline controller
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_enable = true;
    break;

  case 10:
    //dynamic deceleration (line)
    if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * DECELERATION_KP);
    }
    if(speed > 130.0)
      speed = 130.0;
    if(speed < 100.0)
      speed = 100.0;
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;
    break;

  case 11:
    //start trackwall controller, set trackwall params
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("track_wall_controller");
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,100.0,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    switch_enable = true;
    dywall_enable = true;
    break;

  case 12:
    //dynamic deceleration (wall)
    if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * DECELERATION_KP);
    }
    if(speed > 130.0)
      speed = 130.0;
    if(speed < 100.0)
      speed = 100.0;
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,speed,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    dywall_enable = true;
    back_speed = speed;
    break;

  case 13:
    //start moveit controller
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;

  case 14:
    //turn left
    rotation_Xangle(0,90,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 15:
    //at M_PI * 90/180 stop
    rotation_Xangle(1,90,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 16:
    //start track wall, speed = 170, slope low
    wallrange = right_length;
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_wall_controller");
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,170.0,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    dywall_enable = true;
    switch_enable = true;
    break;

  case 17:
    //slow down speed, speed = 100
    SetWallDynamicParams(&dynamic_msg, 50.0,0.0,5000.0,100.0,0.3,1.571);
    dynamic_srv.request.config = dynamic_msg;
    dywall_enable = true;
    break;

  case 18:
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;

  case 19:
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
    //change controller duration
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
    //wait duration
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
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_2_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 3:
    //wait duration
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

  case 4:
    //change controller duration
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

  case 5:
    //sensor almost white
    if(get_sensor_average() < TASK_5_SENSOR_THROSHOLD)
      return true;
    break;

  case 6:
    //change controller duration
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

  case 7:
    // Detect the Orient is stable at 0 degree and no line
    if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * (-90.0/180));
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      return true;
    }
//    //yaw > 90
//    if(yaw > TASK_13_DEG_THROSHOLD)
//      return true;
    break;

  case 8:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_8_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 9:
    //change controller duration
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
    //sensor all black
    if(get_sensor_average() > TASK_10_SENSOR_THROSHOLD)
      return true;
    break;

  case 11:
    //change controller duration
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

  case 12:
    //scan front
    if(front_length < TASK_12_SCAN_FRONT_THROSHOLD)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      return true;
    }
    break;

  case 13:
    //change controller duration
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

  case 14:
    // Detect the Orient is stable at 90 degree and no line
    if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * 175.0/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      return true;
    }
    break;

  case 15:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_15_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 16:
    //change controller duration
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_16_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 17:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_17_WAIT_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 18:
    //change controller duration
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

  case 19:

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
