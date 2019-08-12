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
#define SENSOR_REG_COUNT (7)
#define SIZE_DATA_RECOARD (10)
#define CONVERG_THROSHOLD (M_PI * 1.0/180.0)

#define SWITCH_CONTROLLER_DURATION    (0.2)
#define turndeg_kp (30.0f)
#define turndeg_kd (180.0f)
#define DECELERATION_KP  (40)
#define ORIENT_RIGHT_KP  (100.0f)
#define ORIENT_RIGHT_KD  (3400.0f)
#define ORIENT_plus_RIGHT_KP  (50.0f)
#define ORIENT_plus_RIGHT_KD  (700.0f)

#define TASK_1_WAIT_DURATION         (1.0)
#define TASK_2_WAIT_DURATION         (1.2)
#define TASK_3_WAIT_DURATION         (0.5)
#define TASK_8_WAIT_DURATION         (0.1)
#define TASK_15_WAIT_DURATION        (0.2)
#define TASK_16_WAIT_DURATION        (2.5)
#define TASK_17_WAIT_DURATION        (1.0)

#define TASK_5_SENSOR_THROSHOLD      (90)
#define TASK_10_SENSOR_THROSHOLD     (200)

#define TASK_7_DEG_THROSHOLD         (90.0f)
#define TASK_14_DEG_THROSHOLD        (90.0f)

#define TASK_12_SCAN_FRONT_THROSHOLD  (0.7f)
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
float pitch(0.0);
float pitch_base(0.0);
float front_length(10.0);
float right_length(10.0);
float left_length(10.0);
int L_counter(0);

int stage(20);

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
  for (int i = 0; i < 7; i++) {
    sensor_value[i] = ptr[i];
  }
  is_sensor_ready = true;
}
void callback_IMU_yaw(const std_msgs::Float32ConstPtr& msg){
  yaw = msg->data;
  is_imu_ready = true;
}
void callback_IMU_pitch(const std_msgs::Float32ConstPtr& msg){
  pitch = msg->data;
  //is_imu_ready = true;
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
  ros::init(argc, argv, "test_main_loop2");
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
  ros::Subscriber IMU_pitch_sub = node.
      subscribe<std_msgs::Float32>("/imu/pitch", 50, callback_IMU_pitch);
  ros::Subscriber scan_sub = node.
      subscribe<sensor_msgs::LaserScan>("/scan", 50, callback_scan);

  /******************************** Publishers ********************************/
  ros::Publisher move_it_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",50);


  ros::Rate r(30);

  printf("set stage:\n");  
  scanf("%d",&stage);

  while (ros::ok())
  {
    //if(is_sensor_ready && is_laser_ready){
    if(is_laser_ready  && is_imu_ready && is_sensor_ready){
    ///////////////////////////////////////////////////////
    ///// if stage change do changeControllers() ones /////
    ///////////////////////////////////////////////////////
      if(!is_call){
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
      if(stage == 2){
        //turn deg
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 5){
        //fuzzy decelerate
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if(stage == 12){
        //track wall
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 14){
        //turn deg
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 16) {
        //track wall (slope up)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 17){
        //track wall (on slope)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 171){
        //track wall (slope down)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 21){
        //
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 22){
        //
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 24){
        //
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 25){
        //
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 27){
        //track wall (slope up)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 28){
        //track wall (on slope)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 999291){
        //find black line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 999293){
        //black line ing
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 2935){
        //
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 294){
        //decelerate (slope down)
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 301){
        //find S start
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 302){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 201){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 19){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 191){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 192){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 193){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 194){
        //on white line
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }else if (stage == 36){
        //turn deg
        changeControllers(stage, &funcase_client, &IMU_zero_client, &move_it_pub, &dynamic_line_client, &dynamic_wall_client);
      }
    ///////////////////////////////////////////////////////
    ///////////// repeat changeControllers() //////////////
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    ////////////// check and change stage /////////////////
    ///////////////////////////////////////////////////////
      if(stage == 39){
//      if(stage == 295){//295){
        //stop at stage XXX
      }else if(stage ==5){
          if(stage_change_detect(stage)){
            stage = 11;
            is_call = false;
          }
      }else if(stage ==2){
          if(stage_change_detect(stage)){
            stage = 201;
            is_call = false;
          }
      }else if(stage ==201){
          if(stage_change_detect(stage)){
            stage = 4;
            is_call = false;
          }
      }else if(stage == 12){
          if(stage_change_detect(stage)){
            stage = 14;
            is_call = false;
          }
      }else if(stage == 17){
          if(stage_change_detect(stage)){
            stage = 171;
            is_call = false;
          }
      }else if(stage == 173){
          if(stage_change_detect(stage)){
            stage = 18;
            is_call = false;
          }
      }else if(stage == 19){
        if(stage_change_detect(stage)){
            stage = 191;
            is_call = false;
          }
      }else if(stage == 194){
        if(stage_change_detect(stage)){
            stage = 20;
            is_call = false;
          }
      }else if(stage == 27){
        if(stage_change_detect(stage)){
            stage = 29;
            is_call = false;
          }
      }else if(stage == 29){
        if(stage_change_detect(stage)){
            stage = 290;
            is_call = false;
          }
      }else if(stage == 293){
        if(stage_change_detect(stage)){
            stage = 2931;
            is_call = false;
          }
      }else if(stage == 2935){
        if(stage_change_detect(stage)){
            stage = 295;
            is_call = false;
          }
      }else if(stage == 295){
        if(stage_change_detect(stage)){
            stage = 301;
            is_call = false;
          }
      }else if(stage == 301){
        if(stage_change_detect(stage)){
            stage = 31;
            is_call = false;
          }
      }else if(stage == 999){
        if(stage_change_detect(stage)){
            stage = 666;
            is_call = false;
          }
      }else if(stage == 9992934){
        if(stage_change_detect(stage)){
            stage = 666;
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
    ROS_INFO("sensor_value[6]: %d",sensor_value[6]);
    ROS_INFO("yaw: %4.3f pitch: %4.3f",yaw,pitch);
    ROS_INFO("stage: %d",stage);
    ROS_INFO("right_length: %4.3f front_length: %4.3f left_length: %4.3f",right_length,front_length,left_length);
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
    

    turn = static_cast<int16_t>(turndeg_kp * cot_angle(yaw-(M_PI *_angle/180.0)));
    if(turn >= 0 && turn < 90)
      turn = 90;
    if(turn < 0 && turn > -90)
      turn = -90;
    _move_msg->data.push_back(-turn);
      _move_msg->data.push_back(turn);
    /*if(turn > 0){
      _move_msg->data.push_back(turn+80);
      _move_msg->data.push_back(-turn-80);
    }else{
      _move_msg->data.push_back(turn-80);
      _move_msg->data.push_back(-turn+80);
    }*/
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
    moveit_msg.data.push_back(110);
    pubmsg_enable = true;
    break;

  case 2:
    //turn right
    //moveit_msg.data.push_back(-120);
    //moveit_msg.data.push_back(120);

    error = -cot_angle(yaw-(M_PI *(-90.0)/180.0));
    error_dot = error - error_back;
    error_back= error;
    turn = static_cast<int16_t>(turndeg_kp*error + turndeg_kd*error_dot);
    if(turn > 0){
      moveit_msg.data.push_back(turn+100);
      moveit_msg.data.push_back(-turn-125);
      printf("l speed: %d\n", turn+100);
      printf("r speed: %d\n", -turn-125);
    }else{
      moveit_msg.data.push_back(turn-100);
      moveit_msg.data.push_back(-turn+125);
      printf("l speed: %d\n", turn-100);
      printf("r speed: %d\n", -turn+125);
    }
    printf("error: %4.3f\n",error);
    printf("error dot: %4.3f\n",error_dot);
    printf("turn: %d\n", turn);
    pubmsg_enable = true;
    break;

  case 201:
    error = -(cot_angle(yaw - (M_PI *(-90.0)/180.0))) + (0.6f - get_right_distence(cot_angle(yaw - (M_PI *(-90.0)/180.0))))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(100+turn);
    moveit_msg.data.push_back(100-turn+25);
    pubmsg_enable = true;

    break;
  case 3:
    //move farward
    moveit_msg.data.push_back(140);
    moveit_msg.data.push_back(140);
    pubmsg_enable = true;
    break;

  case 666:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;

  case 667:
    moveit_msg.data.push_back(200);
    moveit_msg.data.push_back(255);
    pubmsg_enable = true;
    break;

  case 4:
    //start trackline controller, set trackline params
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,200.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    switch_enable = true;
    back_speed = 200.0;
    break;

  case 5: 

    //dynamic deceleration (line)
    if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * DECELERATION_KP);
    }
    if(speed > 200.0)
      speed = 200.0;
    if(speed < 70.0)
      speed = 70.0;  
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;
    break;

/*  case 6:
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
    break;*/

  case 11:
    //start trackwall controller, set trackwall params
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");

    wallrange = right_length;
    //setzeo_enable = true;
    switch_enable = true;
    break;

  case 12:
    //         ORIENT_RIGHT_KP              TASK_10_LENGTH_RIGHT

    if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * 80);
    }
    if(speed > 200.0)
      speed = 200.0;
    if(speed < 70.0)
      speed = 70.0;

    error = -(cot_angle(yaw)) + (wallrange - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(speed+turn);
    moveit_msg.data.push_back(speed-turn+25);
    pubmsg_enable = true;
    break;

  case 13:
    //start moveit controller
    switch_control.request.stop_controllers.push_back("track_wall_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    break;

   case 14:
    //turn left
    error = -cot_angle(yaw-(M_PI *90.0/180.0));
    error_dot = error - error_back;
    error_back= error;
    turn = static_cast<int16_t>(turndeg_kp*error + turndeg_kd*error_dot);

    if(turn > 0){
      moveit_msg.data.push_back(turn+100);
      moveit_msg.data.push_back(-turn-125);
      printf("l speed: %d\n", turn+100);
      printf("r speed: %d\n", -turn-100);
    }else{
      moveit_msg.data.push_back(turn-100);
      moveit_msg.data.push_back(-turn+125);
      printf("l speed: %d\n", turn-100);
      printf("r speed: %d\n", -turn+100);
    }

    printf("error: %4.3f\n",error);
    printf("error dot: %4.3f\n",error_dot);
    printf("turn: %d\n", turn);

    pubmsg_enable = true;
    break;

  case 15:
    //at M_PI * 90/180 stop
    //wallrange = right_length;
    rotation_Xangle(1,90,&moveit_msg);
    pubmsg_enable = true;
    wallrange = right_length;
    setzeo_enable = true;
    pitch_base = pitch;
    break;

  case 16:
    error = -(cot_angle(yaw)) + (wallrange - get_right_distence(cot_angle(yaw)))*3;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_plus_RIGHT_KP*error + ORIENT_plus_RIGHT_KD*error_dot);
    //moveit_msg.data.push_back(200);
    //moveit_msg.data.push_back(230);
    if(turn > 0){
      moveit_msg.data.push_back(170+turn);
      moveit_msg.data.push_back(210); 
      printf("r speed: %d\n", 160+turn);
      printf("l speed: %d\n\n", 190); 
    }else{
      moveit_msg.data.push_back(170);
      moveit_msg.data.push_back(210-turn);
      printf("r speed: %d\n", 160);
      printf("l speed: %d\n\n", 190-turn);
    }
    pubmsg_enable = true;
   
    
    break;

  case 17:
    error = -(cot_angle(yaw)) + (0.4 - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(75+turn);
    moveit_msg.data.push_back(75-turn+10);
    pubmsg_enable = true;
    break;   

  case 171:
    //slope down
    error = -(cot_angle(yaw)) + (wallrange - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(60+turn);
    moveit_msg.data.push_back(60-turn+10);
    pubmsg_enable = true;
    break;
  
  case 172:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;

  case 173:
    if((left_length < 1.8f) && (get_sensor_average() > 220)){
      moveit_msg.data.push_back(-100);
      moveit_msg.data.push_back(150);
    }else if ((left_length > 1.8f) && (get_sensor_average() > 220)){
      moveit_msg.data.push_back(150);
      moveit_msg.data.push_back(-100);
    }else if(get_sensor_average() < 220){
      moveit_msg.data.push_back(0);
      moveit_msg.data.push_back(0);    
    }
    
    pubmsg_enable = true;

    break;


  
  case 18:
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_enable = true;
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.7,100.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    break;

  case 19:       
    
    back_speed = 200;
    break;
  
  case 191:
     if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * 25);
    }
    if(speed > 200.0)
      speed = 200.0;
    if(speed < 70.0)
      speed = 70.0;   
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;

    break;    
 
 case 192:
     if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * 23);
    }
    if(speed > 200.0)
      speed = 200.0;
    if(speed < 70.0)
      speed = 70.0;   
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;

    break;

  case 193:
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.7,70.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    break;   

  case 194:
     if(isnan(front_length)){
      speed = back_speed;
    }else {
      speed = static_cast<double>(front_length * 60);
    }
    if(speed > 200.0)
      speed = 200.0;
    if(speed < 70.0)
      speed = 70.0;   
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,speed);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    back_speed = speed;

    break; 
  
  case 20:
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    setzeo_enable = true;
    wallrange = right_length;
    break;

  case 21:
    error = -(cot_angle(yaw)) + (wallrange - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(100+turn);
    moveit_msg.data.push_back(100-turn+10);
    pubmsg_enable = true;
    break;

  case 22:
    error = -cot_angle(yaw-(M_PI *90.0/180.0));
    error_dot = error - error_back;
    error_back= error;
    turn = static_cast<int16_t>(turndeg_kp*error + turndeg_kd*error_dot);

    if(turn > 0){
      moveit_msg.data.push_back(turn+100);
      moveit_msg.data.push_back(-turn-125);
      printf("l speed: %d\n", turn+100);
      printf("r speed: %d\n", -turn-100);
    }else{
      moveit_msg.data.push_back(turn-100);
      moveit_msg.data.push_back(-turn+125);
      printf("l speed: %d\n", turn-100);
      printf("r speed: %d\n", -turn+100);
    }

    printf("error: %4.3f\n",error);
    printf("error dot: %4.3f\n",error_dot);
    printf("turn: %d\n", turn);
    pubmsg_enable = true;
    break;

  case 23:
    //at M_PI * 90/180 stop
    //wallrange = right_length;
    rotation_Xangle(1,90,&moveit_msg);
    setzeo_enable = true;
    pubmsg_enable = true;
    break;
  
  case 24:
    if(isnan(front_length)){
      speed = 150;
    }else {
      speed = static_cast<double>(front_length * 35);
    }
    if(speed > 150.0)
      speed = 150.0;
    if(speed < 80.0)
      speed = 80.0;
   
    error = -(cot_angle(yaw)) + (0.45 - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(speed+turn);
    moveit_msg.data.push_back(speed-turn+10);
    pubmsg_enable = true;
    break;

  case 25:
    error = -cot_angle(yaw-(M_PI *90.0/180.0));
    error_dot = error - error_back;
    error_back= error;
    turn = static_cast<int16_t>(turndeg_kp*error + turndeg_kd*error_dot);


    if(turn > 0){
      moveit_msg.data.push_back(turn+100);
      moveit_msg.data.push_back(-turn-125);
      printf("l speed: %d\n", turn+100);
      printf("r speed: %d\n", -turn-100);
    }else{
      moveit_msg.data.push_back(turn-100);
      moveit_msg.data.push_back(-turn+125);
      printf("l speed: %d\n", turn-100);
      printf("r speed: %d\n", -turn+100);
    }
    pubmsg_enable = true;
    break;

  case 26:
    //at M_PI * 90/180 stop
    //wallrange = right_length;
    rotation_Xangle(1,90,&moveit_msg);
    pubmsg_enable = true;
    setzeo_enable = true;
    wallrange = right_length;
    break;

  case 27:
    error = -(cot_angle(yaw)) + (0.4 - get_right_distence(cot_angle(yaw)))*3;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_plus_RIGHT_KP*error + ORIENT_plus_RIGHT_KD*error_dot);

    if(turn > 0){
      moveit_msg.data.push_back(235+turn);
      moveit_msg.data.push_back(255); 
      printf("r speed: %d\n", 160+turn);
      printf("l speed: %d\n\n", 190); 
    }else{
      moveit_msg.data.push_back(235);
      moveit_msg.data.push_back(255-turn);
      printf("r speed: %d\n", 160);
      printf("l speed: %d\n\n", 190-turn);
    }
    pubmsg_enable = true;
 
    printf("error: %4.3f\n",error);
    printf("error2: %4.3f\n",wallrange - get_right_distence(cot_angle(yaw)));
    printf("error dot: %4.3f\n",error_dot);
    printf("turn: %d\n", turn);
    break;

  case 28:
    error = -(cot_angle(yaw)) + (wallrange - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(100+turn);
    moveit_msg.data.push_back(100-turn+10);
    pubmsg_enable = true;
    break;
  
  case 29:
    moveit_msg.data.push_back(-220);
    moveit_msg.data.push_back(-220);
    pubmsg_enable = true;

  case 290:
    printf("********* %4.3f ***********", get_right_distence(cot_angle(yaw)));
    printf("********* %4.3f ***********", get_front_distence(cot_angle(-yaw)));
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;

  case 291:
    if(get_right_distence(cot_angle(yaw)) > 0.4){
      moveit_msg.data.push_back(110);
      moveit_msg.data.push_back(-110);
    }else{
      moveit_msg.data.push_back(110);
      moveit_msg.data.push_back(-150);
    }
    pubmsg_enable = true;
    break;

  case 292:
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_enable = true;
    SetLineDynamicParams(&dynamic_msg, -0.5,0.0,-5.0,85.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    break;

  case 293:
    printf("L_counter = %d\n", L_counter);
    break;
  
  case 2931:
    printf("L_counter = %d\n", L_counter);
    break;
    
  case 2932:
    printf("L_counter = %d\n", L_counter);
    break;

  case 2933:
     switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    //
    //setzeo_enable = true;
    //
    break;
  
  case 2934:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;

  case 2935:
    if(isnan(front_length)){
      speed = 100;
    }else {
      speed = static_cast<double>(front_length * 35);
    }
    if(speed > 100)
      speed = 100;
   
    error = -(cot_angle(yaw)) + (0.4 - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(150*error + 4000*error_dot);
    moveit_msg.data.push_back(static_cast<int16_t>(speed)+turn);
    moveit_msg.data.push_back(static_cast<int16_t>(speed)-turn+10);
    printf("******* turn: %d ************", turn);
    printf("********* l speed: %d   r speed: %d ************",static_cast<int16_t>(speed)+turn,static_cast<int16_t>(speed)-turn+10);
    pubmsg_enable = true;
    break;

  case 294:
    SetLineDynamicParams(&dynamic_msg, -0.5,0.0,-6.0,40.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    break;

  case 295:
    //switch_control.request.stop_controllers.push_back("track_line_controller");
    //switch_control.request.stop_controllers.push_back("move_it_controller");
    //switch_enable = true;
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enable = true;
    break;

  case 296:
    error = -cot_angle(yaw) + (0.4f - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(70+turn);
    moveit_msg.data.push_back(70-turn+25);
    pubmsg_enable = true;

 
  case 301:
    /*printf("********* %4.3f ***********", get_right_distence(cot_angle(yaw)));
    printf("********* %4.3f ***********", get_front_distence(cot_angle(-yaw)));
    if(get_right_distence(cot_angle(yaw)) > 0.41){
      moveit_msg.data.push_back(70);
      moveit_msg.data.push_back(130+10);
    }else{
      moveit_msg.data.push_back(130);
      moveit_msg.data.push_back(70+10);    
    }
    pubmsg_enable = true;*/
    error = -(cot_angle(yaw)) + (0.4 - get_right_distence(cot_angle(yaw)))*5;
    error_dot = error - error_back;
    error_back= error;

    turn = static_cast<int16_t>(ORIENT_RIGHT_KP*error + ORIENT_RIGHT_KD*error_dot);
    moveit_msg.data.push_back(70+turn);
    moveit_msg.data.push_back(70-turn+25);
    pubmsg_enable = true;
    break;

  case 302:
    moveit_msg.data.push_back(-40);
    moveit_msg.data.push_back(-100);
    pubmsg_enable = true;
    break;

  case 31:
    switch_control.request.stop_controllers.push_back("move_it_controller");
    switch_control.request.start_controllers.push_back("track_line_controller");
    SetLineDynamicParams(&dynamic_msg, 0.7,0.0007,10.0,150.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    switch_enable = true;
    //
    //setzeo_enable = true;
    //
    break;

  case 32:
    break;

  case 33:
    SetLineDynamicParams(&dynamic_msg, 0.2,0.0,4.5,100.0);
    dynamic_srv.request.config = dynamic_msg;
    dyline_enable = true;
    break;

  case 34:
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_enable = true;
    break;
 
  case 35:
    moveit_msg.data.push_back(100);
    moveit_msg.data.push_back(110);
    pubmsg_enable = true;
    break;

  case 36:
   error = -cot_angle(yaw-(M_PI *(-90.0)/180.0));
    error_dot = error - error_back;
    error_back= error;
    turn = static_cast<int16_t>(turndeg_kp*error + turndeg_kd*error_dot);

    if(turn > 0){
      moveit_msg.data.push_back(turn+100);
      moveit_msg.data.push_back(-turn-125);
      printf("l speed: %d\n", turn+100);
      printf("r speed: %d\n", -turn-125);
    }else{
      moveit_msg.data.push_back(turn-100);
      moveit_msg.data.push_back(-turn+125);
      printf("l speed: %d\n", turn-100);
      printf("r speed: %d\n", -turn+125);
    }

    printf("error: %4.3f\n",error);
    printf("error dot: %4.3f\n",error_dot);
    printf("turn: %d\n", turn);
    pubmsg_enable = true;
    break;

  case 37:
    //at M_PI * 90/180 stop
    //wallrange = right_length;
    rotation_Xangle(1,90,&moveit_msg);
    pubmsg_enable = true;
    break;

  case 38:
    moveit_msg.data.push_back(-100);
    moveit_msg.data.push_back(-120);
    pubmsg_enable = true;
    break;

  case 39:
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
  static int pitch_counter(0);
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
      if((ros::Time::now().toSec() - last_time.toSec() > TASK_1_WAIT_DURATION) && (sensor_value[6] == 3)){
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    //sensor all black
    //if(get_sensor_average() > 200)
    //if(sensor_value[6] == 3)
    //  return true;
    break;

  case 2:
    //wait duration
    /*if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((ros::Time::now().toSec() - last_time.toSec() > TASK_2_WAIT_DURATION) && (sensor_value[5] < 20)) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }*/
    // Detect the Orient is stable at 90 degree and no line
    if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * (-90.0)/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;


  case 201:
   if(right_length > 0.7f)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;
    
  case 3:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((ros::Time::now().toSec() - last_time.toSec() > TASK_3_WAIT_DURATION) && (sensor_value[6] == 3)){
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 667:
    //wait 2.0S
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 2.0) {
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

    if((cot_angle(fabs(yaw)) < M_PI * 10.0/180.0) && (!fg_usetimer)){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((ros::Time::now().toSec() - last_time.toSec() > 0.5f)){
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    
    break;

 /* case 6:
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
    break;*/

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
      error_dot = 0.0;
      error_back= 0.0;
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
      stable_detector.init(M_PI * 90.0/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      error_dot = 0.0;
      error_back= 0.0;
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
    /*if(!fg_usetimer){
      pitch_base = pitch;
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(std::fabs(pitch - pitch_base) > 1.0f) {
       pitch_counter++;
      }
    }
    if(pitch_counter > 2 ) {
      pitch_counter = 0;
      fg_usetimer = false;
      last_time = ros::Time::now();
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }*/
    if(!fg_usetimer && (pitch < -2.0))
      fg_usetimer = true;
    if(fg_usetimer){
//      if(pitch > -1.0){
      printf("pitch %4.3f***************************\n",pitch);
      if(pitch>-2.0){
        fg_usetimer = false;
        error_dot = 0.0;
        error_back= 0.0;
        return true;
      }
    }
    break;

  case 17:
   
    if(std::abs(pitch) >= 2){
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;

  case 171:
//    if(right_length > 1.0){
    if(sensor_value[6] == 3){
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;

  case 172:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 173:
    if(yaw > M_PI*30.0/180.0 && yaw < M_PI*150.0/180.0){
      if(sensor_value[2] < 50 || sensor_value[3] < 50)    
        return true;
    }


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
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(sensor_value[6] == 1) {
        if(ros::Time::now().toSec() - last_time.toSec() > 2.0f){
          fg_usetimer = false;
          last_time = ros::Time::now();
          error_dot = 0.0;
          error_back= 0.0;
          return true;
        }
      }
    }
    break;


  case 191:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(sensor_value[6] == 1) {
        if(ros::Time::now().toSec() - last_time.toSec() > 1.0f){
          fg_usetimer = false;
          last_time = ros::Time::now();
          error_dot = 0.0;
          error_back= 0.0;
          return true;
        }
      }
    }
    break;

  case 192:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(sensor_value[6] == 1) {
        if(ros::Time::now().toSec() - last_time.toSec() > 1.0f){
          fg_usetimer = false;
          last_time = ros::Time::now();
          error_dot = 0.0;
          error_back= 0.0;
          return true;
        }
      }
    }
    break;

  case 193:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(sensor_value[6] == 2) {
        if(ros::Time::now().toSec() - last_time.toSec() > 1.0f){
          fg_usetimer = false;
          last_time = ros::Time::now();
          error_dot = 0.0;
          error_back= 0.0;
          return true;
        }
      }
    }
    break;
  case 194:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((front_length < 1.5f) &&(right_length < 0.70f) && (left_length < 0.7f)) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 20:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > SWITCH_CONTROLLER_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        error_dot = 0.0;
        error_back= 0.0;
        return true;
      }
    }
    break;

     case 21:
    //scan front
    if(front_length < 0.6f)
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      
      return true;
    }
    break;

  case 22:
    // Detect the Orient is stable at 90 degree and no line
    if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * 90.0/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      return true;
    }
    break;

  case 23:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 0.1) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

   case 24:
    //scan front
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        if(front_length < 0.6f)
          laser_distence_overlimit_conter++;
      }
    }
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      fg_usetimer = false;
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;

  case 25:
    // Detect the Orient is stable at 90 degree and no line
    if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * (90.0)/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      return true;
    }
    break;

  case 26:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        error_dot = 0.0;
        error_back= 0.0;
        return true;
      }
    }
    break;
  
  case 27:
    //slope up
    if(!fg_usetimer && (pitch < -5.0))
      fg_usetimer = true;
    if(fg_usetimer){
//      if(pitch > -1.0){
      printf("pitch %4.3f***************************\n",pitch);
      if(pitch>-2.0){
        fg_usetimer = false;
        error_dot = 0.0;
        error_back= 0.0;
        return true;
      }
    }

    break;

  case 28:
    //slope down
    if(!fg_usetimer && pitch > 5.0)
      fg_usetimer = true;
    if(fg_usetimer){
      if(laser_distence_overlimit_conter == 0){
//        if(sensor_value[6] == 3)
//        if(get_sensor_average() > 200)
        if(pitch < 1.0)
          laser_distence_overlimit_conter = 1;
      }else if(laser_distence_overlimit_conter == 1){
        if(sensor_value[6] == 1 || sensor_value[6] ==2){
          laser_distence_overlimit_conter = 0;
          error_dot = 0.0;
          error_back= 0.0;
          return true;
        }
      }
    }
    break;
  
  case 29:
    //stop
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 0.05f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    return true;
    break;

  case 290:
    //wait
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;
  
  case 291:
    if(yaw > M_PI*30.0/180.0 && yaw < M_PI*150.0/180.0){
      if(sensor_value[2] > 200 || sensor_value[3] > 200)    
        return true;
    }
      //if(((yaw >= -1) && (yaw <= 1)) || ((yaw <= 1.9) && (yaw >= 1.3)))  
      //  return true;
    break;

  case 292:
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

  case 293:
    if((yaw >= M_PI*80.0/180.0) && (yaw <= M_PI*100.0/180.0))
      return true;
     /*if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){ 
      if(sensor_value[6] == 2) {
        if(ros::Time::now().toSec() - last_time.toSec() > 1.0f){
          L_counter++;
          last_time = ros::Time::now();
        }
      }
      if(L_counter >= 2){
        fg_usetimer = false;
        L_counter = 0;
        last_time = ros::Time::now();
        return true;     
      }
    }*/
    break;

  case 2931:
    if((yaw >= M_PI*(-100.0)/180.0) && (yaw <= M_PI*(-80.0)/180.0))
      return true;
    /*if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){ 
      if(sensor_value[6] == 1) {
        if(ros::Time::now().toSec() - last_time.toSec() > 1.0f){
          L_counter++;
          last_time = ros::Time::now();
        }
      }
      if(L_counter >= 1){
        fg_usetimer = false;
        L_counter = 0;
        last_time = ros::Time::now();
        return true;     
      }
    }*/
    break;

  case 2932:
    if((yaw >= -M_PI*6.0/180.0) && (yaw <= M_PI*6.0/180.0))
      L_counter++;
    if(L_counter > 5){
      L_counter = 0;
      return true;
    }
    /*if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.5f){ 
        if((yaw >= -M_PI*10.0/180.0) && (yaw <= M_PI*10.0/180.0)){      
          fg_usetimer = false;
          last_time = ros::Time::now();
          return true;  
        } 
      }
    }*/
    break;

  case 2933:
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
  
  case 2934:
    //wait
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 0.7) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;
 
  case 2935:
    /*if(!fg_usetimer && pitch > 2.5 ){
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(front_length < 1.5f)
        laser_distence_overlimit_conter++;
    }
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      fg_usetimer = false;
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }*/

  if(!fg_usetimer && pitch > 2.5 ){
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(sensor_value[6] == 3){
        fg_usetimer = false;
        error_dot = 0.0;
        error_back= 0.0;
        return true;
      }
    }

  case 294:
    if(sensor_value[6] == 3)
      return true;
    break;

  case 295:
    //wait
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 296:
    //change controller duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(front_length < 0.6f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 301:
    if(front_length < 1.0f){
        laser_distence_overlimit_conter++;
    }
    if(laser_distence_overlimit_conter > 2 ) {
      laser_distence_overlimit_conter = 0;
      error_dot = 0.0;
      error_back= 0.0;
      return true;
    }
    break;

  case 302:
    //fund s line
    if(sensor_value[2] < 30 || sensor_value[3] < 30)
      return true;
    break;

  case 31:
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

  case 32:
    printf("************* %4.3f *********",fabs(cot_angle(yaw-(M_PI *180.0/180.0))));
    /*if((fabs(cot_angle(yaw-(M_PI *180.0/180.0))) < M_PI * 7.0/180.0) && (right_length < 0.5))
      return true;*/

    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 20.0) {
        if((fabs(cot_angle(yaw-(M_PI *180.0/180.0))) < M_PI * 5.0/180.0) && (right_length < 0.6)){
          fg_usetimer = false;
          last_time = ros::Time::now();
          return true;
        }
      }
    }
    break;

  case 33:
    if(sensor_value[6] == 1)
      return true;
    break;

  case 34:
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

  case 35:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((ros::Time::now().toSec() - last_time.toSec() > 0.08f)){
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break; 

  case 36:
    /*if(!stable_detector.isStarted()) {
      stable_detector.init(M_PI * (90.0)/180);
      stable_detector.start();
    } else {
      stable_detector.update();
    }
    if(stable_detector.isConverged()) {
      stable_detector.stop();
      return true;
    }*/

    if(fabs(yaw + (M_PI*90.0/180.0)) < (M_PI*2.0/180.0))
      laser_distence_overlimit_conter++;
    if(laser_distence_overlimit_conter == 20){
      laser_distence_overlimit_conter = 0;
      return true;
    }
    break; 

  case 37:
    //wait duration
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if(ros::Time::now().toSec() - last_time.toSec() > 1.0f) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 38:
    if(!fg_usetimer){
      last_time = ros::Time::now();
      fg_usetimer = true;
    }
    if(fg_usetimer){
      if((sensor_value[6] == 0)){
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
    for(int i=0; i<SENSOR_REG_COUNT-1; i++) {
        sum += sensor_value[i];
    }

    return sum / (SENSOR_REG_COUNT-1);
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

//2019 08 03
