#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "controller_manager_msgs/SwitchController.h"
//#include <joy_msg/JoyCmd.h>
//#include "funcase_client/JoyCmd.h"
using namespace std;
/***********************************************/

int stage(0);
bool is_call(false);
int joys[16]={ 0 };
int hands[9]={0};
int abs_hands[9]={128,128,128,90,180,128,128,128,60};
int handsMAX[9]={193,250,200,176,180,182,207,170,130};
int handsMIN[9]={75,0,50,0,150,71,48,46,0};
int handspeed=2;
void changeControllers(int _stage, ros::ServiceClient* _funcase_client);
void move_pub(int lspeed,int rspeed);
void arm_pub(int Rcatch,int RJ1,int RJ2,int RJ3,int RJ4,int Lcatch,int LJ1,int LJ2,int LJ3);
int JointMaxMin(int AX,int max ,int min);
/***********************************************/
void fake_arm(const std_msgs::Int16MultiArrayConstPtr& msgs)
{
  for(int i =0;i<4;i++)
  {
  hands[i]=(msgs->data.at(i))/4;
}
}
void joy_decode(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  /**/
  int b,b1,b2,b3,b4,b5,b6,b7,b8,n;
      const uint8_t* ptr = msg->data.data();
      //NO.1 item
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
     //NO.2,3 item
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

/*
       for(int j=0;j<16;j++)
       {
         cout<<joys[j]<<",";

       }
       cout<<endl;
       */
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
  subscribe<std_msgs::UInt8MultiArray>("/joy_commands", 1, joy_decode);
  ros::Subscriber hand_sub = node.
  subscribe<std_msgs::Int16MultiArray>("hand_pose", 1, fake_arm);

  /******************************** Publishers ********************************/
  ros::Publisher move_it_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/move_it_controller/move_it",1);
  ros::Publisher cam_pub = node.
      advertise<std_msgs::Bool>("/switch_camera",1);
  ros::Publisher sound_pub = node.
      advertise<std_msgs::Bool>("/BZ5",1);
  ros::Publisher hand_pub = node.
      advertise<std_msgs::Int16MultiArray>("/funcasebot/arm_controller/move_arm",10);
  std_msgs::Bool cam_type,sound_type;
  std_msgs::UInt8 soundandalert;




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
      //POWER COMMAND

      if(joys[7]==1){
        is_call = false;
        stage++;
      }/*stop condition*/
      break;
    case 1:
      int lspeed=0,rspeed=0;
      if(!is_call){
        changeControllers(1, &funcase_client);
        ROS_INFO("stage 1 : Contraller is Running");
        is_call = true;
      }
      ROS_INFO("stage 1 : Waiting for sending remote pose");

      //left joys
      if(joys[13]==0&&joys[12]==0)
      {
         if(joys[9]==1 && joys[8]==-1){
					 lspeed=110;
           rspeed=150;
				   ROS_INFO("stage 1 : FrontRight");
         }else if(joys[9]==1 && joys[8]==1){
           lspeed=150;
           rspeed=110;
				   ROS_INFO("stage 1 : FrontLeft");
         }else if(joys[9]==-1 && joys[8]==-1){
					 lspeed=-110;
           rspeed=-220;
				   ROS_INFO("stage 1 : backRight");
         }else if(joys[9]==-1 && joys[8]==1){
           lspeed=-220;
           rspeed=-110;
				   ROS_INFO("stage 1 : backLeft");
         }else if(joys[9]==-1){
           lspeed=-130;
           rspeed=-130;
				   ROS_INFO("stage 1 : back");
				 }else if(joys[8]==-1){
       	   lspeed=-130;
       	   rspeed=150;
       	   ROS_INFO("stage 1 : Right");
      	 }else if(joys[9]==1){
           lspeed=110;
           rspeed=140;
				   ROS_INFO("stage 1 : Front");
				 }else if(joys[8]==1){
      	   lspeed=130;
     	     rspeed=-130;
     	     ROS_INFO("stage 1 : Left");
      	 }
    	}
      //CAMSWITCH

      if(joys[5]==1)
      {
      cam_type.data=1;
      ROS_INFO("stage 1 : Cam has been switched");
      cam_pub.publish(cam_type);
      }
      else if(joys[6]==1)
      {
      cam_type.data=0;
      ROS_INFO("stage 1 : Cam has been switched");
      cam_pub.publish(cam_type);
      }
      //L HAND POSE PUB

      if(joys[12]==1)
      {
        if(joys[7]==1)
        {

      ROS_INFO("stage 1 : LeftHand posing");
      arm_pub(abs_hands[0],abs_hands[1],abs_hands[2],abs_hands[3],abs_hands[4],hands[5],hands[6],hands[7],hands[8]);
        }

      }


      //L HAND POSE Plus
      if(joys[12]==1 && joys[7]!=1)
      {
        //JOINT1
        if(joys[9]==1)
        {
          abs_hands[5]=abs_hands[5]+handspeed;

        ROS_INFO("stage 1 : 1st Joint RH^^");

        }
        else if(joys[9]==-1)
        {
          abs_hands[5]=abs_hands[5]-handspeed;
         ROS_INFO("stage 1 : 1st joint RHVV");
        }
        //JOINT2
        if(joys[8]==-1)
        {
        abs_hands[6]=abs_hands[6]+handspeed;
        ROS_INFO("stage 1 : 2nd Joint RH^^");

        }
        else if(joys[8]==1)
        {
        abs_hands[6]=abs_hands[6]-handspeed;
        ROS_INFO("stage 1 : 2nd joint RHVV");
        }
        //JOINT3
        if(joys[11]==1)
        {
        abs_hands[7]=abs_hands[7]+handspeed;
        ROS_INFO("stage 1 : 3th Joint RH^^");

        }
        else if(joys[11]==-1)
        {
        abs_hands[7]=abs_hands[7]-handspeed;
        ROS_INFO("stage 1 : 3th joint RHVV");
        }

        //
        if(joys[14]==1)
        {
        abs_hands[8]=90;
        ROS_INFO("stage 1 :RH CLOSE");

        }
        else if(joys[14]==-1)
        {
        abs_hands[8]=48;
        ROS_INFO("stage 1 :RH LOSS");
        }

        //
        for(int i=0;i<9;i++)
          {
      abs_hands[i]=JointMaxMin(abs_hands[i],handsMAX[i],handsMIN[i]);
          }
      arm_pub(abs_hands[0],abs_hands[1],abs_hands[2],abs_hands[3],abs_hands[4],abs_hands[5],abs_hands[6],abs_hands[7],abs_hands[8]);
      }



      //R HAND POSE Plus
      if(joys[13]==1 && joys[7]!=1)
      {
        //JOINT1
        if(joys[9]==1)
        {
          abs_hands[0]=abs_hands[0]+handspeed;

        ROS_INFO("stage 1 : 1st Joint RH^^");

        }
        else if(joys[9]==-1)
        {
          abs_hands[0]=abs_hands[0]-handspeed;
         ROS_INFO("stage 1 : 1st joint RHVV");
        }
        //JOINT2
        if(joys[8]==-1)
        {
        abs_hands[1]=abs_hands[1]+handspeed;
        ROS_INFO("stage 1 : 2nd Joint RH^^");

        }
        else if(joys[8]==1)
        {
        abs_hands[1]=abs_hands[1]-handspeed;
        ROS_INFO("stage 1 : 2nd joint RHVV");
        }
        //JOINT3
        if(joys[11]==1)
        {
        abs_hands[2]=abs_hands[2]+handspeed;
        ROS_INFO("stage 1 : 3th Joint RH^^");

        }
        else if(joys[11]==-1)
        {
        abs_hands[2]=abs_hands[2]-handspeed;
        ROS_INFO("stage 1 : 3th joint RHVV");
        }
        //JOINT4
        if(joys[10]==1)
        {
        abs_hands[3]=abs_hands[3]+handspeed;
        ROS_INFO("stage 1 : 4th Joint RH^^");

        }
        else if(joys[10]==-1)
        {
        abs_hands[3]=abs_hands[3]-handspeed;
        ROS_INFO("stage 1 : 4th joint RHVV");
        }
        //
        if(joys[14]==1)
        {
        abs_hands[4]=180;
        ROS_INFO("stage 1 :RH CLOSE");

        }
        else if(joys[14]==-1)
        {
        abs_hands[4]=150;
        ROS_INFO("stage 1 :RH LOSS");
        }

        //
        for(int i=0;i<9;i++)
          {
      abs_hands[i]=JointMaxMin(abs_hands[i],handsMAX[i],handsMIN[i]);
          }
      arm_pub(abs_hands[0],abs_hands[1],abs_hands[2],abs_hands[3],abs_hands[4],abs_hands[5],abs_hands[6],abs_hands[7],abs_hands[8]);
      }





      //R HAND POSE PUB

      if(joys[13]==1)
      {
        if(joys[7]==1)
        {
          ROS_INFO("stage 1 : RightHand posing");
          arm_pub(hands[0],hands[1],hands[2],hands[3],hands[4],abs_hands[5],abs_hands[6],abs_hands[7],abs_hands[8]);
          for(int i=0;i<5;i++)
          {
            abs_hands[i]=hands[i];
          }
        }

      }
      //sound
      if(joys[4]==1)
      {
      sound_type.data=1;
      ROS_INFO("stage 1 : let START the music");
      sound_pub.publish(sound_type);
      }else{
      sound_type.data=0;
      ROS_INFO("stage 1 : let STOP the music");
      sound_pub.publish(sound_type);
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
   move_it_pub.publish(move);
}
/**/
int JointMaxMin(int AX,int max ,int min)
{
  if(AX>=max)
  {
    AX=max;
  }
  if(AX<=min)
  {
    AX=min;
  }
  return AX;
}




/*ARMPUB*/
void arm_pub(int Rcatch,int RJ1,int RJ2,int RJ3,int RJ4,int Lcatch,int LJ1,int LJ2,int LJ3)
{
  int y[9]={Rcatch,RJ1,RJ2,RJ3,RJ4,Lcatch,LJ1,LJ2,LJ3};
  std_msgs::Int16MultiArray arm;
   ros::NodeHandle node;
   ros::Publisher hand_pub = node.
       advertise<std_msgs::Int16MultiArray>("/funcasebot/arm_controller/move_arm",10);
  for(int i=0;i<9;i++)
  {
   arm.data.push_back(y[i]);
  }
   hand_pub.publish(arm);
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
    switch_control.request.start_controllers.push_back("arm_controller");
    break;
  case 2:
    break;
  }

  _funcase_client->call(switch_control);
}
