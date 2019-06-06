#ifndef TRACK_WALL_CONTROLLER_H
#define TRACK_WALL_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_msgs/JointControllerState.h>

namespace funcase_controllers
{
  class TrackWallController : public controller_interface::
                    Controller<hardware_interface::EffortJointInterface>
  {
  public:
    TrackWallController();
    ~TrackWallController();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& node);
    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool read_parameter();

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
    //track lidar sub
    ros::Subscriber track_lidar_sub;

    //param for PID
    int error;
    int error_sum;
    int error_dot;
    int error_back;

    double initspeed;
    double turn;
    double k_p;
    double k_i;
    double k_d;
  };
}

#endif // TRACK_WALL_CONTROLLER_H
