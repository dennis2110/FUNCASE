#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "controller_manager_msgs/SwitchController.h"

/****************    Config   *******************/
#define TASK_0_DURATION         (0.2)
#define TASK_1_DURATION         (2.0)
#define TASK_2_DURATION         (2.0)
#define TASK_3_DURATION         (2.0)
/************************************************/

/***********************************************/
float front_length(10.0);
float right_length(10.0);
float left_length(10.0);

int stage(0);
bool is_call(false);
uint8_t sensor_value[4] = { 0 };
float yaw(0.0);
double aaa(0.0);

sensor_msgs::LaserScan laser_msg;

void changeControllers(int _stage, ros::ServiceClient* _funcase_client, ros::Publisher* _funcase_moveit_pub);
bool stage_change_detect(int _stage);

/******************* callback ****************************/

void callback_sensor(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  const uint8_t* ptr = msg->data.data();
  for (int i = 0; i < 4; i++) {
    sensor_value[i] = ptr[i];
  }
}
void callback_IMU_yaw(const std_msgs::Float32ConstPtr& msg){
  yaw = msg->data;
}
void callback_scan(const sensor_msgs::LaserScan msg){
 laser_msg = msg;

 float length_sum(0.0);
 for(int i=0;i<9;i++)
   length_sum += msg.ranges.at(i+535);
 right_length = length_sum / 9;

 length_sum = 0.0;
 for(int i=0;i<9;i++)
   length_sum += msg.ranges.at(i+355);
 front_length = length_sum / 9;

 length_sum = 0.0;
 for(int i=0;i<9;i++)
   length_sum += msg.ranges.at(i+175);
 left_length = length_sum / 9;
}

/***********************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotcontrol_auto_client");
  ros::NodeHandle node;

  /********************************** Client **********************************/
  ros::ServiceClient funcase_client = node.
      serviceClient<controller_manager_msgs::SwitchController>("/funcasebot/controller_manager/switch_controller");

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
    /**** check stage ****/
    if(!is_call){
      changeControllers(4, &funcase_client, &move_it_pub);
      ROS_INFO("stage %d", stage);
      is_call = true;
    }
    ROS_INFO("left_length: %4.3f front_length: %4.3f right_length: %4.3f", left_length, front_length, right_length);
//    if(is_call){
//      std_msgs::Int16MultiArray moveit_msg;
//      moveit_msg.data.push_back(100);
//      moveit_msg.data.push_back(100);
//      move_it_pub.publish(moveit_msg);
//    }

    if(stage_change_detect(stage)){
        stage++;
        is_call = false;
    }

    /*********************/
    //std_msgs::Int16MultiArray move_cmd;
    //move_cmd.data.push_back(10);
    //ROS_INFO("pub %d", move_cmd.data.at(0));
    //ROS_INFO("callback %d", sensor_value[0]);
    /*********************/
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
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_control.request.stop_controllers.push_back("track_line_controller");
    switch_enable = true;
    break;

  case 1:
    moveit_msg.data.push_back(80);
    moveit_msg.data.push_back(80);
    pubmsg_enalbe = true;
    break;

  case 2:
    moveit_msg.data.push_back(80);
    moveit_msg.data.push_back(-80);
    pubmsg_enalbe = true;
    break;

  case 3:
    moveit_msg.data.push_back(-80);
    moveit_msg.data.push_back(-80);
    pubmsg_enalbe = true;
    break;

  case 4:
    moveit_msg.data.push_back(0);
    moveit_msg.data.push_back(0);
    pubmsg_enalbe = true;
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
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_0_DURATION) {
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
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_1_DURATION) {
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
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_1_DURATION) {
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
      if(ros::Time::now().toSec() - last_time.toSec() > TASK_1_DURATION) {
        fg_usetimer = false;
        last_time = ros::Time::now();
        return true;
      }
    }
    break;

  case 4:

    break;
  }

  // Default False
  return false;
}
