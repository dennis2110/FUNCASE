#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

//#define LIDAR_DEBUG
#define laser_sample_num 720
#define laser_start      159
#define laser_end        201
#define  pi 3.14159265

using namespace std;

float laser_msg[laser_sample_num] = {};
float x[43]={},y[43]={},r_save,angle_save;

void hough_transform(float ave_laser[], float &_r_save, float& _angle_save);
void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
#ifdef LIDAR_DEWBUG
  //ROS_INFO("angle_max[%4.3f] angle_min[%4.3f] range_max[%4.3f] range_min[%4.3f] scan_time[%4.3f] angle_increment[%4.3f] time_increment[%4.3f]"
  //           , msg->angle_max, msg->angle_min, msg->range_max, msg->range_min, msg->scan_time, msg->angle_increment, msg->time_increment);
  ROS_INFO("laser_sample_num: %d", static_cast<int>(msg->ranges.size()));
#endif
  for(size_t i=0;i<laser_sample_num;i++){
    laser_msg[i] = msg->ranges.at(i);
  }
  hough_transform(laser_msg, r_save, angle_save);
  //ROS_INFO("r_save: %4.3f angle_save: %4.3f", r_save, angle_save);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_hough_transform");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/scan", 1, lidarCallback);

  ros::spin();

  return 0;
}

void hough_transform(float ave_laser[], float &_r_save, float& _angle_save){
  float laser_value[43];
  int laser_num, laser_locate[43];

  int acc_arr[41][41]={};
  float acc_num[41];

  float max_x=0, rho;

  float err,err_min = 100;
  int err_num;

  int r_angle[2];
  int ticket_max=0;
  //int out=0;

  int cnt = 0;
  for(int i=0;i<43;i++){
    if(ave_laser[i+laser_start]>= static_cast<float>(0.1)){
      laser_value[cnt] = ave_laser[i+laser_start];
      laser_locate[cnt]= i+laser_start;
      cnt++;
    }
  }
  laser_num = cnt;
  ////////////////////////////////////////////////////////
  //if(out == 0){
    for(int i=0;i<laser_num;i++){
      x[i] = laser_value[i]*cos((laser_locate[i]-180)*0.5*pi/180);
      y[i] = laser_value[i]*sin((laser_locate[i]-180)*0.5*pi/180);
      if(max_x < ave_laser[i]){
        max_x = laser_value[i];
      }
    }
    ROS_INFO("max_x: %4.3f", max_x);
    //segmentation rho to 41 resolution
    for(int i=0;i<41;i++){
      acc_num[i] = -max_x+(max_x*2/40)*i;
    }

    for(int i=0;i<laser_num;i++){
      for(int j=0;j<41;j++){
        rho = x[i]*cos(j*(pi/40))+y[i]*sin(j*(pi/40));
        //ROS_INFO("%d rho: %4.3f", j, rho);
        //if(r > 0){
        for(int k=0;k<41;k++){
          err = abs(acc_num[k]- rho) ;
          if(err_min > err){
            err_min = err;
            err_num = k;
            //cout << "acc_num" << k << ":" << acc_num[k] << endl;
          }
          //cout << "err" << k << ":" << err[k] << endl;
        }
        //}
        err_min=100;
        acc_arr[err_num][j]++;
        //cout << "r" << i << ":" << r << endl;

      }
    }

    for(int i=0;i<41;i++){
      for(int j=0;j<41;j++){
        if(ticket_max <acc_arr[j][i]){
          ticket_max=acc_arr[j][i];
          r_angle[0]=j;
          r_angle[1]=i;
        }
        //cout << "acc_arr" << i << j << ":" << acc_arr[i][j] << endl;
      }
    }
    _r_save = acc_num[r_angle[0]];
    _angle_save = r_angle[1]*(pi/40);
    ROS_INFO("final_r: %4.3f  final_angle: %4.3f", _r_save, _angle_save);

  //}

}
