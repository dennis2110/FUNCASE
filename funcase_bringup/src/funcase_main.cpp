#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include "funcase_hw.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "funcase_main");
  ros::NodeHandle node;
  ros::NodeHandle node2;
  ros::CallbackQueue queue;
  node.setCallbackQueue(&queue);

  FuncaseRobot robot;
  robot.init(&node2);
  controller_manager::ControllerManager cm(&robot, node);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  //ros::Rate r(1.0 / robot.getPeriod().toSec());
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read();
     cm.update(ts, d);
     robot.write();
     rate.sleep();
  }

  spinner.stop();
  return 0;
}
