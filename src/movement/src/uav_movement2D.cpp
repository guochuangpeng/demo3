#include "uav_movement2D.h"

//仍需考虑输出速度时的偏航问题
rel_controller::rel_controller()
{   
  ros::NodeHandle private_nh_("~"); 
  private_nh_.param<double>("u_max", u_max, 0.75);//最大输出速度设置
  private_nh_.param<double>("target_height", target_height, 0.0);//目标的高度设置
  private_nh_.param<double>("para_beta", para_beta, 8.0);//速度控制器参数
  private_nh_.param<double>("para_C", para_C, 1.0);//速度控制器参数
  private_nh_.param<double>("highest_height", highest_height, 3.0);//最大飞行高度
  private_nh_.param("visual_enable", visual_enable, true);//启用视觉
  private_nh_.param<double>("d_safe", d_safe, 2.0);//docking安全距离
  //private_nh_.param<double>("set_height", set_height, 2.28);//设置飞行高度
  /******激光测高参数******/
  //private_nh_.param("distance_sensor_range", distance_sensor_range, 0.28);//传感器安装位置距离无人机最下层的高度
  //private_nh_.param("lidar_data_enable", lidar_data_enable, true);//use lidar data be z pose//激光雷达测高使能参数

  //height_sub = n_.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 1000, &rel_controller::height_callback, this);//激光测高 
  local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &rel_controller::local_pos_cb,this);//订阅本地位置信息  ENU坐标系  ~30hz
  uwb_rel_pos_sub = nh_.subscribe<relative_localization::relative_position>("/relative_position/uwb/pos_result", 10, &rel_controller::uwb_rel_pos_cb,this);//订阅uwb相对位置信息 ENU  ~33hz
  visual_rel_pos_sub = nh_.subscribe<visual_estimation::visual_rel_pos>("/relative_position/visual/pos_result", 10, &rel_controller::visual_rel_pos_cb,this);//订阅visual相对位置信息
  Hope_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);//发布输出速度信息   机体坐标系  ~33hz
  Hope_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);//发布输出位置信息  NED坐标系

  //ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  //ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");//Service中的 Client节点  用作mavros外部控制
  //ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  /*****************************uav parameters*********************************/
  uwb_phase = true;//uwb驱动阶段
  visual_phase = false;//视觉驱动阶段
  uav_ready = false;//无人机准备完毕与否
  should_land = false;
  //high_sensor_value = -5;//cm 激光测高高度参数初始化
  close_target = false;
  detection_success = false;
  
  UAV_rotate = 0.0;
  UAV_x = 0.0;
  UAV_y = 0.0;
  UAV_z = 0.0;
}    

void rel_controller::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //无人机实时高度
  UAV_z = msg->pose.position.z;//单位：m
  //无人机坐标
  UAV_x = msg->pose.position.x;
  UAV_y = msg->pose.position.y;

  //四元数参数
  double quaternion_x = msg->pose.orientation.x;
  double quaternion_y = msg->pose.orientation.y;
  double quaternion_z = msg->pose.orientation.z;
  double quaternion_w = msg->pose.orientation.w;

  Eigen::Quaterniond q;
  q.x() = quaternion_x;
  q.y() = quaternion_y;
  q.z() = quaternion_z;
  q.w() = quaternion_w;
  Eigen::Matrix<double,3,3>rotMat = q.normalized().toRotationMatrix();
     
  //根据旋转矩阵求出坐标旋转角
  double theta_x = atan2(rotMat(2, 1), rotMat(2, 2));
  double theta_y = atan2(-rotMat(2, 0), sqrt(rotMat(2, 1)*rotMat(2, 1) + rotMat(2, 2)*rotMat(2, 2)));
  double theta_z = atan2(rotMat(1, 0), rotMat(0, 0));
   
  //将弧度转化为角度
  theta_x = theta_x * (180 / PI);
  theta_y = theta_y * (180 / PI);
  theta_z = theta_z * (180 / PI);
  UAV_rotate = theta_z;//偏航角度，ENU
  //ROS_INFO("UAV_rotate:%f",UAV_rotate);
  //std::cout<<"UAV_rotate:"<<UAV_rotate<<std::endl;
}

/*
void rel_controller::height_callback(const sensor_msgs::Range::ConstPtr& msg)
{
  if(lidar_data_enable==true)
  {
    static double lidar_delta_time_begin = ros::Time::now().toSec();
    static double lidar_delta_time_end   = ros::Time::now().toSec();
    static double lidar_delta_t = 0.1;
    static int lidar_delta_t_count = 0;
      
    ////////////TODO:check distance_sensor_range
    uav_height = (msg->range-distance_sensor_range)*1000;//uav底部到地面的距离
      
    lidar_delta_t_count++;
    if(lidar_delta_t_count>=50)//10hz,~5s
    {
      lidar_delta_time_end   = ros::Time::now().toSec();
      lidar_delta_t = (lidar_delta_time_end-lidar_delta_time_begin)/100;
      lidar_delta_time_begin = ros::Time::now().toSec();
      lidar_delta_t_count = 0;
      if(lidar_delta_t>1)lidar_delta_t=1;
      //ROS_INFO("%f",lidar_delta_t);0.018
    }
      
    Eigen::MatrixXd lidar_range(2,1);
      
    rangekf.setThreshold(-1);
    rangekf.setSingleSensorR(20);// 20cm
    lidar_range = rangekf.filter(Eigen::MatrixXd::Identity(1,1)*(msg->range*100),init,lidar_delta_t,0);
    rangekf.showState();
      
    high_sensor_value = -msg->range*100;
    
    pos.z_pos = uav_height;
  }
  if(pose.pose.position.x!=0&&pose.pose.position.y!=0)//当无人机不在初始位置时
  {
    if(!uav_ready)//当无人机ready状态false时
    {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = false;//解除uav的offboard模式
      if( arming_client.call(arm_cmd) )
      {
        ROS_INFO("UAV ready!");
        uav_ready = true;
      }
    }
  }
}
*/


void rel_controller::uwb_rel_pos_cb(const relative_localization::relative_position::ConstPtr& msg)
{
  distance = msg->uwb_distance_2dimension;//单位：m
  q_rel_uwb(0,0) = msg->x_pos_relative;//ENU坐标系
  q_rel_uwb(1,0) = msg->y_pos_relative;//ENU坐标系
  uav_height = msg->z_pos;//uav底部到地面的距离
  high_sensor_value = msg->lidar_value;
  para_sigma(0,0) = msg->sigma1;
  para_sigma(1,0) = msg->sigma2;
  if( distance < d_safe ){//距离小于d_safe转为视觉定位模式
    //if(!visual_phase){
      vel_uwb.linear.x = 0.0;
      vel_uwb.linear.y = 0.0;
      //vel_uwb.linear.z = 0.0;
      vel_uwb.linear.z = (-0.2) * exp( (0.1)*UAV_z );//downward
      Hope_vel_pub.publish(vel_uwb);//发布速度信息
      //ROS_INFO("Sending Zero velocity and Turn to Visual Phase");
      ROS_INFO("velocity: vel_x:%f vel_y:%f vel_z:%f", vel_uwb.linear.x, vel_uwb.linear.y, vel_uwb.linear.z);
   // }
    uwb_phase = false;
    visual_phase = true;
  }
  if(uwb_phase){ 
    u_relative = ( (-1.0) * para_beta * q_rel_uwb ) + ( para_C * distance * para_sigma );//速度控制器
    u_relative_2_norm = sqrt( (u_relative(0,0)*u_relative(0,0)) + (u_relative(1,0)*u_relative(1,0)) );
    if(u_max>u_relative_2_norm){
      u_out = u_relative;//实际输出速度
    }
    else{
      u_out = u_max * u_relative / u_relative_2_norm;//实际输出速度 ENU
    }
    /**************ENU TO 机体坐标系****************/
    vel_x = (u_out(0,0) * cos(UAV_rotate * PI / 180.0)) + (u_out(1,0) * sin(UAV_rotate * PI / 180.0));
    vel_y = ((-1.0) * u_out(0,0) * sin(UAV_rotate * PI / 180.0)) + (u_out(1,0) * cos(UAV_rotate * PI / 180.0));
    vel_uwb.linear.x = vel_y;
    vel_uwb.linear.y = (-1.0) * vel_x;
    //vel_uwb.linear.z = 0.0;
    vel_uwb.linear.z = (-0.1) * (UAV_z-1.6);
    /*********************************************/
    Hope_vel_pub.publish(vel_uwb);//发布速度信息
    ROS_INFO("Sending velocity");
    ROS_INFO("velocity: vel_x:%f vel_y:%f vel_z:%f", vel_uwb.linear.x, vel_uwb.linear.y, vel_uwb.linear.z);
  }
}

void rel_controller::visual_rel_pos_cb(const visual_estimation::visual_rel_pos::ConstPtr& msg)
{
  if(visual_enable){
    q_rel_visual(0,0) = msg->x_pos_relative;
    q_rel_visual(1,0) = msg->y_pos_relative;

    if( q_rel_visual(0,0) == -1 && q_rel_visual(1,0) == -1 ){// —1为检测失败
      detection_success = false;
    }
    else{
      detection_success = true;
    }

    if(visual_phase && detection_success && !should_land){
      //图像坐标系转换为机体坐标系
      double q_x = (-1.0)*q_rel_visual(1,0);
      double q_y = (-1.0)*q_rel_visual(0,0);
      //转换后的期望位移 机体坐标系转换为ENU坐标系
      double q_x_ENU = ((-1.0)*q_x * cos(UAV_rotate * PI / 180.0)) - (q_y * sin(UAV_rotate * PI / 180.0));
      double q_y_ENU = ((-1.0)*q_x * sin(UAV_rotate * PI / 180.0)) + (q_y * cos(UAV_rotate * PI / 180.0));
      //ENU坐标系转换为NED坐标系
      double q_x_NED = q_y_ENU;
      double q_y_NED = q_x_ENU;
      //无人机应该移动到的位置坐标  UAV_x是ENU？？？
      double s_x = UAV_x + q_x_NED;
      double s_y = UAV_y + q_y_NED;
      pos_visual.pose.position.x = s_x;
      pos_visual.pose.position.y = s_y;
      Hope_pos_pub.publish(pos_visual);//发布位置信息
      ROS_INFO("Sending position");
      ROS_INFO("position: s_x:%f s_y:%f", pos_visual.pose.position.x, pos_visual.pose.position.y);
    }

    /**************************landing阶段**************************/
    if(!should_land){
      int steady_count = 0;
      if(distance < 0.1){
        close_target = true;
        steady_count++;
        if(steady_count > 10){
          should_land = true;
          ROS_INFO("Should land");
        }
      }
      else{
        steady_count = 0;
      }
    }

    if(should_land){
      if(close_target){//接近目标时
        if(detection_success){//检测到视觉基准
          if(UAV_z > 0.2){//高度大于20cm时，下降
            ROS_INFO("Quadrotor z coordinate: %.2f", UAV_z);
            ROS_INFO("Sending downward velocity");
            vel_uwb.linear.x = 0.0;
            vel_uwb.linear.y = 0.0;
            vel_uwb.linear.z = (-0.5) * exp( (0.1)*UAV_z );
            Hope_vel_pub.publish(vel_uwb);
          }
          else{//高度小于20cm时，输出0速度
            ROS_INFO("Quadrotor z coordinate: %.2f", UAV_z);
            ROS_INFO("Sending zero velocity");
            vel_uwb.linear.x = 0.0;
            vel_uwb.linear.y = 0.0;
            vel_uwb.linear.z = 0.0;
            Hope_vel_pub.publish(vel_uwb);
          }
        }
        else{//检测不到视觉基准，上升    上升过程中即转回visual_phase 重新检测
          if(UAV_z < highest_height){
            ROS_INFO("Quadrotor z coordinate: %.2f", UAV_z);
            ROS_INFO("Sending upward velocity");
            vel_uwb.linear.z = (0.5) * exp( (0.1)*UAV_z );
            Hope_vel_pub.publish(vel_uwb);  
            should_land = false;
          }
          else{
            ROS_INFO("Reached highest height");
            vel_uwb.linear.z = 0;
            Hope_vel_pub.publish(vel_uwb);
            should_land = false;
          }
        }
      }
      else{//偏离目标时
        should_land = false;
      }
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_movement2D");
  rel_controller position;
  ros::spin();
  /*ros::Rate r(10); 
  while(ros::ok()){
     // read
     ros::spinOnce();
     r.sleep();
  }*/
}

