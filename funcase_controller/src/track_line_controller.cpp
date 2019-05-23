#include "track_line_controller.h"

funcase_controllers::TrackLineController::TrackLineController(){

}

funcase_controllers::TrackLineController::~TrackLineController(){

}

bool funcase_controllers::TrackLineController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){

}

void funcase_controllers::TrackLineController::update(const ros::Time &time, const ros::Duration &period){

}

void funcase_controllers::TrackLineController::starting(const ros::Time &time){

}

void funcase_controllers::TrackLineController::stopping(const ros::Time &time){

}

bool funcase_controllers::TrackLineController::read_parameter(){

}

void funcase_controllers::TrackLineController::setCommand(uint8_t sensor1, uint8_t sensor2, uint8_t sensor3, uint8_t sensor4){

}

void funcase_controllers::TrackLineController::setCommandCB(const std_msgs::UInt8MultiArrayConstPtr &sensor_msg){

}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackLineController, controller_interface::ControllerBase)
