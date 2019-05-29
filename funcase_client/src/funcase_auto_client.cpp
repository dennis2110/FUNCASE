#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"

/***********************************************/

int stage(0);
bool is_call(false);
uint8_t sensor_value[4] = { 0 };


void changeControllers(int _stage, ros::ServiceClient* _funcase_client);

/***********************************************/

void callback_sensor(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  const uint8_t* ptr = msg->data.data();
  for (int i = 0; i < 4; i++) {
    sensor_value[i] = ptr[i];
  }
}

/***********************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "funcase_auto_client");
  ros::NodeHandle node;

  /********************************** Client **********************************/
  ros::ServiceClient funcase_client = node.
      serviceClient<controller_manager_msgs::SwitchController>("/funcasebot/controller_manager/switch_controller");

  /******************************** Subscriber ********************************/
  ros::Subscriber sub = node.
      subscribe<std_msgs::UInt8MultiArray>("/track_line_sensor", 50, callback_sensor);

  /******************************** Publishers ********************************/
  ros::Publisher move_it_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",50);



  ros::Rate r(30);

  while (ros::ok()) {
    /**** check stage ****/
    ROS_INFO("%d",stage);
    switch (stage) {
    case 0:
      if(!is_call){
        changeControllers(stage, &funcase_client);
        ROS_INFO("stage 0");
        is_call = true;
      }
      if(sensor_value[0]>150 && sensor_value[3]>150){
        is_call = false;
        stage++;
      }
      break;
    case 1:
      changeControllers(stage, &funcase_client);
      stage++;
      ROS_INFO("stage 1");
      break;
    case 2:
      ROS_INFO("stage 2");
      break;
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

void changeControllers(int _stage, ros::ServiceClient* _funcase_client){
  /*** change stage ****/
  controller_manager_msgs::SwitchController switch_control;

  switch (_stage) {
  case 0:
    switch_control.request.start_controllers.push_back("track_line_controller");
    switch_control.request.stop_controllers.push_back("move_it_controller");
    break;
  case 1:
    switch_control.request.start_controllers.push_back("move_it_controller");
    switch_control.request.stop_controllers.push_back("track_line_controller");
    break;
  case 2:
    break;
  }

  _funcase_client->call(switch_control);
}
