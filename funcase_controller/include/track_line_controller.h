#ifndef TRACK_LINE_CONTROLLER_H
#define TRACK_LINE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_msgs/JointControllerState.h>

#include <std_msgs/UInt8MultiArray.h>
#include <boost/scoped_ptr.hpp>

namespace funcase_controllers
{
  class TrackLineController : public controller_interface::
                    Controller<hardware_interface::EffortJointInterface>
  {
  public:
    TrackLineController();
    ~TrackLineController();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& node);
    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool read_parameter();
    void setCommand(uint8_t sensor1,uint8_t sensor2,uint8_t sensor3,uint8_t sensor4);
    void setCommandCB(const std_msgs::UInt8MultiArrayConstPtr& sensor_msg);
  public:
    // current node
    ros::NodeHandle m_node;
    /* registered robot and joints */
    hardware_interface::EffortJointInterface* m_robot;
    hardware_interface::JointHandle m_left_wheel;
    hardware_interface::JointHandle m_right_wheel;
    //for joint state
    hardware_interface::JointStateInterface* m_funcase_state;
    hardware_interface::JointStateHandle m_l_funcase_state;
    hardware_interface::JointStateHandle m_r_funcase_state;


  private:
    hardware_interface::JointHandle joint_;

    //track sensor sub
    ros::Subscriber track_sensor_sub;

    uint8_t sensor_data[4];
  };
}



#endif // TRACK_LINE_CONTROLLER_H
