#ifndef MOVE_IT_CONTROLLER_H
#define MOVE_IT_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Int16MultiArray.h>

namespace funcase_controllers
{
  class MoveItController : public controller_interface::
                   Controller<hardware_interface::EffortJointInterface>
  {
  public:
    MoveItController();
    ~MoveItController();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& node);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool read_parameter();
    void setMoveCommand(int16_t left_vel, int16_t right_vel);
    void setMoveCommandCB(const std_msgs::Int16MultiArrayConstPtr& move_msg);

  public:
    struct MoveCommands
    {
      double l_wheel_vel;
      double r_wheel_vel;
    };
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

    MoveCommands movecmd_struct;

  private:
    hardware_interface::JointHandle joint_;

    //move cmd sub
    ros::Subscriber move_cmd_sub;
  };
}

#endif // MOVE_IT_CONTROLLER_H
