#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
//#include <joy_msg/JoyCmd.h>
#include "funcase_msgs/JoyCmd.h"
using namespace std;
/***********************************************/

int stage(0);
bool is_call(false);
int joys[16]={ 0 };
void changeControllers(int _stage, ros::ServiceClient* _funcase_client);
void move_pub(int lspeed,int rspeed);
/***********************************************/

void joy_decode(const funcase_msgs::JoyCmdConstPtr& msg)
{
  /**/
  int b,b1,b2,b3,b4,b5,b6,b7,b8,n;
      const uint8_t* ptr = msg->joy.data();
        n=ptr[0];
       b1=(n&128)>0?1:0;
       joys[0]=b1;
       b2=(n&64)>0?1:0;
       joys[1]=b2;
       b3=(n&32)>0?1:0;
       joys[2]=b3;
       b4=(n&16)>0?1:0;
       joys[3]=b4;
       b5=(n&8)>0?1:0;
       joys[4]=b5;
       b6=(n&4)>0?1:0;
       joys[5]=b6;
       b7=(n&2)>0?1:0;
       joys[6]=b7;
       b8=(n&1)>0?1:0;
       joys[7]=b8;
      for(int i=1;i<3;i++)
      {
      n=ptr[i];
      b1=(n&128)>0?1:0;
      b2=(n&64)>0?1:0;
      if (b1==1)
      {joys[i*4+4]=-1;}
      else
      {joys[i*4+4]=b2;
      }
      b3=(n&32)>0?1:0;
      b4=(n&16)>0?1:0;
      if (b3==1)
      {joys[i*4+5]=-1;}
      else
      {joys[i*4+5]=b4;
      }
      b5=(n&8)>0?1:0;
      b6=(n&4)>0?1:0;
      if (b5==1)
      {joys[i*4+6]=-1;}
      else
      {joys[i*4+6]=b6;
      }
      b7=(n&2)>0?1:0;
      b8=(n&1)>0?1:0;
      if (b7==1)
      {joys[i*4+7]=-1;}
      else
      {joys[i*4+7]=b8;
      }
      }


//       for(int j=0;j<16;j++)
//       {
//         cout<<joys[j]<<",";

//       }
       cout<<endl;
      }




/***********************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "funcase_remoterobot_client");
  ros::NodeHandle node;

  /********************************** Client **********************************/
  ros::ServiceClient funcase_client = node.
      serviceClient<controller_manager_msgs::SwitchController>("/funcasebot/controller_manager/switch_controller");

  /******************************** Subscriber ********************************/
  ros::Subscriber sub = node.
      //subscribe<std_msgs::UInt8MultiArray>("/joycommands", 50, joy_decode);
  subscribe<funcase_msgs::JoyCmd>("/joy_commands", 50, joy_decode);

  /******************************** Publishers ********************************/
  ros::Publisher move_it_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",50);



  ros::Rate r(30);

  while (ros::ok()) {
    /**** check stage ****/
    ROS_INFO("%d",stage);
    switch (stage) {
    case 0:
      ROS_INFO("stage 0 : Ready for Start the Game");
      if(!is_call){
        changeControllers(0, &funcase_client);
        ROS_INFO("stage 0 : Contraller is Running");
        is_call = true;
      }
      if(joys[7]==1){
        is_call = false;
        stage++;
      }/*stop condition*/
      break;
    case 1:
      int lspeed=0,rspeed=0;
      changeControllers(1, &funcase_client);
      ROS_INFO("stage 1 : Waiting for sending remote pose");
      if(joys[9]==1)
      {
        lspeed=lspeed-100;
        rspeed=rspeed-100;
      ROS_INFO("stage 1 : Front");
      }
      else if(joys[9]==-1)
      {
        lspeed=lspeed+100;
        rspeed=rspeed+100;
       ROS_INFO("stage 1 : back");
    }

      if(joys[8]==1)
      {
        lspeed=lspeed+100;
        rspeed=rspeed-100;
      ROS_INFO("stage 1 : Right");
      }
      else if(joys[8]==-1)
      {
        lspeed=lspeed-100;
        rspeed=rspeed+100;
       ROS_INFO("stage 1 : Left");
    }

      move_pub(lspeed,rspeed);
      break;
     }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
void move_pub(int lspeed,int rspeed)
{
  int y[2]={lspeed,rspeed};
  std_msgs::Int16MultiArray move;
   ros::NodeHandle node;
   ros::Publisher move_it_pub = node.
       advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",50);
  for(int i=0;i<2;i++)
  {
   move.data.push_back(y[i]);
  }
cout<<move<<endl;
      move_it_pub.publish(move);
}
void changeControllers(int _stage, ros::ServiceClient* _funcase_client){
  /*** change stage ****/
  controller_manager_msgs::SwitchController switch_control;

  switch (_stage) {
  case 0:
    switch_control.request.stop_controllers.push_back("track_line_controller");
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
