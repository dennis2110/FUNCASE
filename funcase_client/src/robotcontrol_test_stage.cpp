#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "controller_manager_msgs/SwitchController.h"


#define SHOW_DEBUG

/****************    Config   *******************/
#define SENSOR_REG_COUNT (5)
#define SIZE_DATA_RECOARD (10)
#define CONVERG_THROSHOLD (M_PI * 5.0/180.0)


#define SWITCH_CONTROLLER_DURATION    (0.2)
#define TASK_5_DURATION         (2.0)
#define TASK_9_DURATION         (2.0)

#define TASK_1_SENSOR_THROSHOLD         (30)
#define TASK_11_SENSOR_THROSHOLD        (35)

#define TASK_3_LENGTH_FRONT         (0.45)
#define TASK_7_LENGTH_FRONT         (0.17)

/************************************************/

/***********************************************/
bool is_sensor_ready(false);
bool is_imu_ready(false);
bool is_laser_ready(false);

float front_length(10.0);
float right_length(10.0);
float left_length(10.0);


bool is_call(false);

uint8_t sensor_value[5] = { 0 };
float yaw(0.0);
int stage(0);

sensor_msgs::LaserScan laser_msg;

void changeControllers(int _stage, ros::ServiceClient* _funcase_client, ros::Publisher* _funcase_moveit_pub);
bool stage_change_detect(int _stage);
double get_sensor_average();
double cot_angle(double _degree);

/***********************************************/

/******************** class ********************/
#ifdef yeh_class
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
ConvergDetector stable_detector;
WBDetector wb_detector;
#endif
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
 for(int i=0;i<9;i++)
   length_sum += msg.ranges.at(i+266);
 right_length = length_sum / 9;

 length_sum = 0.0;
 for(int i=0;i<5;i++)
   length_sum += msg.ranges.at(i);
 for(int i=0;i<4;i++)
   length_sum += msg.ranges.at(i+356);
 front_length = length_sum / 9;

 length_sum = 0.0;
 for(int i=0;i<9;i++)
   length_sum += msg.ranges.at(i+86);
 left_length = length_sum / 9;
 is_laser_ready = true;
}

/***********************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotcontrol_test_stage");
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



  ros::Rate r(30);

  while (ros::ok()) {

    if(is_laser_ready && is_sensor_ready){
      /**** check and change stage ****/
      if(stage == 5){

      }else if (stage_change_detect(stage)){
        stage++;
        is_call = false;
      }

      /**** if stage change do changeControllers() ones****/
      if(!is_call){
        changeControllers(stage, &funcase_client, &move_it_pub);
        ROS_INFO("stage %d", stage);
        is_call = true;
      }
  //    // rotation controller
  //    if(stage = XX){
  //      turn = Kp * (nowang - goalang)
  //      std_msgs::Int16MultiArray moveit_msg;
  //      moveit_msg.data.push_back(100);
  //      moveit_msg.data.push_back(100);
  //      move_it_pub.publish(moveit_msg);
  //    }
    }

#ifdef SHOW_DEBUG
    ROS_INFO("left_length: %4.3f front_length: %4.3f right_length: %4.3f", left_length, front_length, right_length);
    ROS_INFO("controller get sensor : %d %d %d %d %d",sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3],sensor_value[4]);
#endif
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

void changeControllers(int _stage, ros::ServiceClient* _funcase_client, ros::Publisher* _funcase_moveit_pub){
  /*** change stage ****/
  controller_manager_msgs::SwitchController switch_control;
  std_msgs::Int16MultiArray moveit_msg;
  bool switch_enable(false);
  bool pubmsg_enalbe(false);
  switch (_stage) {
  case 0:
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_enable = true;
    break;

  case 1:
    break;

  case 2:
    switch_control.request.start_controllers.push_back("track_wall_controller");
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_enable = true;
    break;

  case 3:
    break;

  case 4:
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_enable = true;
    break;

  case 5:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enalbe = true;
    break;

  case 6:
    switch_control.request.start_controllers.push_back("track_wall_controller");
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;

  case 7:
    break;

  case 8:
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_enable = true;
    break;

  case 9:
    moveit_msg.data.push_back(100);
    moveit_msg.data.push_back(-100);
    pubmsg_enalbe = true;
    break;

  case 10:
    switch_control.request.start_controllers.push_back("track_wall_controller");
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;

  case 11:
    break;
  }

  if(switch_enable)
    _funcase_client->call(switch_control);
  if(pubmsg_enalbe){
    //ros::Duration(0.5).sleep();
    _funcase_moveit_pub->publish(moveit_msg);
  }
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
    //sensor white
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
    //front 0.17
    if(front_length < TASK_3_LENGTH_FRONT)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      return true;
    }
    break;

  case 4:
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
    //90 deg
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_5_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 6:
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
    //front 0.17
    if(front_length < TASK_7_LENGTH_FRONT)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      return true;
    }
    break;

  case 8:
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

  case 9:
    //90 deg
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_9_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 10:
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

  case 11:
    //sensor have value
    if(get_sensor_average() > TASK_11_SENSOR_THROSHOLD)
      return true;
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
