#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Int16MultiArray.h>

#define right_arm_joint_num (5)
#define left_arm_joint_num  (4)

namespace funcase_controllers
{
  class ArmController : public controller_interface::
      Controller<hardware_interface::PositionJointInterface>
  {
  public:
    ArmController();
    ~ArmController();
    bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle& node);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool read_parameter();
    void setArmCommandCB(const std_msgs::Int16MultiArrayConstPtr& arm_msg);

  public:
    struct ArmCommands
    {
      double r_arm_pos[right_arm_joint_num];
      double l_arm_pos[left_arm_joint_num];
    };
    //current node
    ros::NodeHandle m_node;
    //registered robot and joints
    hardware_interface::PositionJointInterface* m_robot;
    std::vector <hardware_interface::JointHandle> m_right_arm;
    std::vector <hardware_interface::JointHandle> m_left_arm;

    //for joint state
//    hardware_interface::JointStateInterface* m_funcase_state;
//    std::vector <hardware_interface::JointStateHandle> m_r_arm_state;
//    std::vector <hardware_interface::JointStateHandle> m_l_arm_state;
    ArmCommands armcmd_struct;

  private:
    hardware_interface::JointHandle joint_;

    //arm cmd sub
    ros::Subscriber arm_cmd_sub;
  };
}

#endif // ARM_CONTROLLER_H
