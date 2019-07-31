#ifndef TRACK_WALL_CONTROLLER_H
#define TRACK_WALL_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_msgs/JointControllerState.h>

#include <dynamic_reconfigure/server.h>
#include "funcase_controllers/TrackWallPIDparamConfig.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h" //add
#include "track_wall_fuzzy.h"
//#include "track_wall_fuzzy_range.h"
//#include "track_wall_fuzzy_twolayer.h"

#define HOUGH_TRANSFORM
#define USE_HOKUYO

#ifdef USE_HOKUYO
  #define laser_sample_num 683
  #define laser_start      1
  #define laser_end        169
  #define laser_range_min  0.1
  #define laser_range_max  1.0
  #define num_of_samples   169
  #define resolution       100
#else
  #define laser_sample_num 360
  #define laser_start      249
  #define laser_end        291
  #define laser_range_min  0.12
#endif

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
    void callback_reconfigure(funcase_controller::TrackWallPIDparamConfig& config, uint32_t level);
    void setCommandCB(const sensor_msgs::LaserScanConstPtr& scan_msg);
    void hough_transform(float ave_laser[], float &_r_save, float& _angle_save);

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
    typedef dynamic_reconfigure::Server<funcase_controller::TrackWallPIDparamConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> dyn_reconf_server_;

    hardware_interface::JointHandle joint_;
    //track lidar sub
    ros::Subscriber track_lidar_sub;
    //pub data
    //ros::Publisher error_pub; //commend
     /*add 7/26*/
    ros::Publisher r_angle_error;
    ros::Publisher r_angle_fuzzy;
    /*add*/

    //param for PID
    float error_angle;
    float error_range;

    float error;
    float error_sum;
    float error_dot;
    float error_back;
    float sum1;
    float sum2;

    float lidar_value[num_of_samples];
    float x[num_of_samples];
    float y[num_of_samples];
    float r_save;
    float angle_save;
    float wallangle;
    float wallrange;

    double initspeed;
    double turn;
    double k_p;
    double k_i;
    double k_d;

    FuzzyCountrol wallfuzzy;
  };
}

#endif // TRACK_WALL_CONTROLLER_H
