#ifndef TRACK_LINE_FUZZY_CONTROLLER_H
#define TRACK_LINE_FUZZY_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_msgs/JointControllerState.h>

#include <std_msgs/UInt8MultiArray.h>
#include <boost/scoped_ptr.hpp>

#include <dynamic_reconfigure/server.h>
#include "funcase_controllers/TrackLinePIDparamConfig.h"

#include "track_line_fuzzy.h"

#define SENSOR_REG_COUNT (6)

namespace funcase_controllers
{
  class TrackLineFuzzyController : public controller_interface::
                    Controller<hardware_interface::EffortJointInterface>
  {

    //dynamic_param server
    //dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig> m_server;

  public:
    TrackLineFuzzyController();
    ~TrackLineFuzzyController();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& node);
    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

  private:
    bool read_parameter();
    void setCommand(uint8_t sensor1,uint8_t sensor2,uint8_t sensor3,uint8_t sensor4,uint8_t sensor5,uint8_t sensor6);
    void setCommandCB(const std_msgs::UInt8MultiArrayConstPtr& sensor_msg);
    void callback_reconfigure(funcase_controller::TrackLinePIDparamConfig& config, uint32_t level);


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
    /// Dynamic Reconfigure server
    typedef dynamic_reconfigure::Server<funcase_controller::TrackLinePIDparamConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> dyn_reconf_server_;

    hardware_interface::JointHandle joint_;

    //track sensor sub
    ros::Subscriber track_sensor_sub;

    uint8_t sensor_data[SENSOR_REG_COUNT];

    //param for fuzzy
    double initspeed;
    double turn;

    LineFuzzyCountrol linefuzzy;
  };
}

#endif // TRACK_LINE_FUZZY_CONTROLLER_H
