#include "track_wall_controller.h"


funcase_controllers::TrackWallController::TrackWallController() :
  error_range(0.0), error_angle(0.0), error_sum(0.0), error_dot(0.0), error_back(0.0), sum1(0.0), sum2(0.0), initspeed(80.0), k_p(0.1), k_i(0.0), k_d(0.0){

}

funcase_controllers::TrackWallController::~TrackWallController(){

}

bool funcase_controllers::TrackWallController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &node){
  m_node = node;
  m_robot = robot;

  // Initialize dynamic_reconfigure server
  funcase_controller::TrackWallPIDparamConfig config;
  config.k_p = 0.4;
  config.k_i = 0.0;
  config.k_d = 0.0;
  config.initspeed = 100.0;
  config.wallrange = 0.35;
  config.wallangle = 1.571;

  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(m_node);
  dyn_reconf_server_->updateConfig(config);
  dyn_reconf_server_->setCallback(boost::bind(&TrackWallController::callback_reconfigure, this, _1, _2));

  // read the parameter at parameter server and registe to class
  if(!read_parameter()) return false;

  track_lidar_sub = m_node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &TrackWallController::setCommandCB, this);
  //error_pub = m_node.advertise<std_msgs::Float64>("/err", 1);//commend
  r_angle_error = m_node.advertise<std_msgs::Float64MultiArray>("/wall_msg", 1);
  r_angle_fuzzy = m_node.advertise<std_msgs::Float64MultiArray>("/wall_fuzzy", 1);


  return true;
}

void funcase_controllers::TrackWallController::update(const ros::Time &time, const ros::Duration &period){
  //error pos turn right
  hough_transform(lidar_value, r_save, angle_save);
  //ROS_INFO("final_r: %4.3f  final_angle: %4.3f",r_save, angle_save);
  //wallfuzzy.nowstatus(angle_save, wallangle, r_save, wallrange);
  //wallfuzzy.fuzzify();
  //turn = static_cast<double>(wallfuzzy.defuzzify());

  /*commend 7/26
  /////////////////////////////////////
  std_msgs::Float64 err_msg;
  err_msg.data = angle_save;
  error_pub.publish(err_msg);
  /////////////////////////////////////
  */ 

   //add
   for(int i =0;i<5;i++){
    printf("lidar[%d]=%4.3f  ",i+1,lidar_value[i]);
   }   
   printf("\n");/*
   for(int i =5;i<10;i++){
    printf("lidar[%d]=%4.3f  ",i+75,lidar_value[i+74]);
   }   
   printf("\n");
   for(int i =10;i<15;i++){
    printf("lidar[%d]=%4.3f  ",i+75,lidar_value[i+74]);
   }   
   printf("\n");*/
   for(int i =164;i<169;i++){
    printf("lidar[%d]=%4.3f  ",i+1,lidar_value[i]);
   }   
   printf("\n");
   //add

  /*add 7/26*/
  float r_angle_arr[5],wall_fuzzy_arr[10];
  r_angle_arr[0] = r_save;
  r_angle_arr[1] = angle_save;
  r_angle_arr[2] = wallfuzzy._err_ran;
  r_angle_arr[3] = wallfuzzy._err_ang;
  r_angle_arr[4] = turn;
  for(int i =0;i<5;i++){
   wall_fuzzy_arr[i]=wallfuzzy.fuzzyError_ran[i];
  }
  for(int i =5;i<10;i++){
   wall_fuzzy_arr[i]=wallfuzzy.fuzzyError_ang[i-5];
  }
  std_msgs::Float64MultiArray r_angle_err,wall_fuzzy;
  
  for(int i =0;i<5;i++){
   r_angle_err.data.push_back(r_angle_arr[i]);
  }
  for(int i =0;i<10;i++){
   wall_fuzzy.data.push_back(wall_fuzzy_arr[i]);
  }
  r_angle_error.publish(r_angle_err);
  r_angle_fuzzy.publish(wall_fuzzy);
  /*add 7/26*/ 

  /*if(turn > 0){
    m_left_wheel.setCommand(initspeed+turn);
    m_right_wheel.setCommand(initspeed);
  }else{
    m_left_wheel.setCommand(initspeed);
    m_right_wheel.setCommand(initspeed-turn);
  }*/
  sum1 = 0.0;
  sum2 = 0.0;
  for (int i=0;i<5;i++){
    if(!isnan(lidar_value[i]))
      sum1 += lidar_value[i];
    if(!isnan(lidar_value[i+164]))
      sum2 += lidar_value[i+164];
  }
  //error = (r_save - 0.35) + (sum2/5 - sum1/5);
  error = sum2/5-wallrange;
  error_sum += error;
  error_dot = error - error_back;
  error_back = error;

  turn = k_p * error + k_i * error_sum + k_d * error_dot;
  std::cout << "left speed->" << initspeed - turn << std::endl;
  std::cout << "right speed->" << initspeed + turn << std::endl;
  std::cout << "error->" << error << ", error_dot->" << error_dot << std::endl;
  std::cout << "r_save->" << r_save <<std::endl;
  std::cout << "turn->" << turn << endl;
printf("\n");
  m_left_wheel.setCommand(initspeed-turn);
  m_right_wheel.setCommand(initspeed+turn);

  
}

void funcase_controllers::TrackWallController::starting(const ros::Time &time){

}

void funcase_controllers::TrackWallController::stopping(const ros::Time &time){
  const double vel = 0.0;
  for (int i = 0; i<2; i++){
    m_left_wheel.setCommand(vel);
    m_right_wheel.setCommand(vel);
  }
}

bool funcase_controllers::TrackWallController::read_parameter(){
  XmlRpc::XmlRpcValue joint_names;
  if(!m_node.getParam("wheels",joint_names)){
    ROS_ERROR("No 'wheel joints' in controller. (namespace: %s)",
                    m_node.getNamespace().c_str());
    return false;
  }
  if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("'wheel joints' is not a struct. (namespace: %s)",
                  m_node.getNamespace().c_str());
    return false;
  }
  XmlRpc::XmlRpcValue name_value;
  name_value = joint_names[0];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
      ROS_ERROR("joints are not strings. (namespace: %s)",
              m_node.getNamespace().c_str());
      return false;
  }

  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
  m_left_wheel=m_robot->
          getHandle((std::string)name_value);
  //joint  l state
  //m_l_turtle_state = m_turtle_state->
  //        getHandle((std::string)name_value);
  //end
  name_value = joint_names[1];
  if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
      ROS_ERROR("wheel joints are not strings. (namespace: %s)",
             m_node.getNamespace().c_str());
      return false;
  }

  ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
  m_right_wheel=m_robot->
          getHandle((std::string)name_value);

  return true;
}

void funcase_controllers::TrackWallController::callback_reconfigure(funcase_controller::TrackWallPIDparamConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request:  kP = %f, ki = %f, kD = %f, initspeed = %f",config.k_p , config.k_i, config.k_d, config.initspeed);
  k_p = config.k_p;
  k_i = config.k_i;
  k_d = config.k_d;
  initspeed = config.initspeed;
  wallrange = static_cast<float>(config.wallrange);
  wallangle = static_cast<float>(config.wallangle);
}

void funcase_controllers::TrackWallController::setCommandCB(const sensor_msgs::LaserScanConstPtr &scan_msg){

#ifdef HOUGH_TRANSFORM
    for (size_t i=laser_start;i<=laser_end;i++) {
      lidar_value[i-laser_start] = scan_msg->ranges.at(i);
    }
#endif
  //ROS_INFO("wall data: %4.3f %4.3f %4.3f", scan_msg->ranges.at(175), scan_msg->ranges.at(179), scan_msg->ranges.at(183));
   
}

void funcase_controllers::TrackWallController::hough_transform(float *ave_laser, float &_r_save, float &_angle_save){
  float laser_value[num_of_samples];
  int laser_num, laser_locate[num_of_samples];

  int acc_arr[resolution+1][resolution+1]={};
  float acc_num[resolution+1];

  float max_x=0, rho;

  float err,err_min = 100;
  int err_num;

  int r_angle[2];
  int ticket_max=0;
  //int out=0;

  int cnt = 0;
  int cnt2= 0;
  for(int i=0;i<num_of_samples;i++){
    if(isnan(ave_laser[i])){
      
    }else{
      laser_value[cnt2] = ave_laser[i];//laser_value[i];
      laser_locate[cnt2]= i+laser_start;//laser_locate[i];
      cnt2++;
    }
  }
  for(int i=0;i<cnt2;i++){
    if((laser_value[i]>= static_cast<float>(laser_range_min)) && (laser_value[i]<= static_cast<float>(laser_range_max))){
      laser_value[cnt] = laser_value[i];
      laser_locate[cnt]= laser_locate[i];
      cnt++;
    }
  }
  laser_num = cnt;
  ////////////////////////////////////////////////////////
  //if(out == 0){
    for(int i=0;i<laser_num;i++){
      x[i] = laser_value[i]*cos((laser_locate[i]+171)*0.3515* M_PIf32 /180);
      y[i] = laser_value[i]*sin((laser_locate[i]+171)*0.3515* M_PIf32 /180);
      if(max_x < ave_laser[i]){
        max_x = laser_value[i];
      }
    }
    //ROS_INFO("max_x: %4.3f", max_x);
    //segmentation rho to 41 resolution
    for(int i=0;i<resolution+1;i++){
      acc_num[i] = -max_x+(max_x*2/resolution)*i;
    }

    for(int i=0;i<laser_num;i++){
      for(int j=0;j<resolution+1;j++){
        rho = x[i]*cos(j*(M_PIf32/resolution))+y[i]*sin(j*(M_PIf32/resolution));
        //ROS_INFO("%d rho: %4.3f", j, rho);
        //if(r > 0){
        for(int k=0;k<resolution+1;k++){
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

    for(int i=0;i<resolution+1;i++){
      for(int j=0;j<resolution+1;j++){
        if(ticket_max <acc_arr[j][i]){
          ticket_max=acc_arr[j][i];
          r_angle[0]=j;
          r_angle[1]=i;
        }
        //cout << "acc_arr" << i << j << ":" << acc_arr[i][j] << endl;
      }
    }
    _r_save = acc_num[r_angle[0]];
    _angle_save = r_angle[1]*(M_PIf32/resolution);

}

PLUGINLIB_EXPORT_CLASS(funcase_controllers::TrackWallController, controller_interface::ControllerBase)
