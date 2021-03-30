#include "relative_localization2D.h"

#define PI 3.1415926

//kalmanFilter rangekf(1,1);

uwb_relative_localization::uwb_relative_localization()
{
   /*****************************param*********************************/
   ros::NodeHandle private_nh_("~"); 
   /******相对定位算法参数******/
   private_nh_.param<double>("target_height", target_height, 1.5);//目标的高度
   private_nh_.param<double>("para_alpha", para_alpha, 10);//para_Gamma = para_alpha * 单位矩阵I作为估计器增益矩阵 
   /******激光测高参数******/
   private_nh_.param<double>("distance_sensor_range", distance_sensor_range, 0.28);//传感器安装位置距离无人机最下层的高度
   
   private_nh_.param("use_pos", use_pos, false);
   private_nh_.param("use_vel", use_vel, true);
   
   /*****************************相对定位参数*********************************/
   //static para
   is_first_step = true;//是否第一步
   //设置初始值，经过算法迭代会收敛到真实值附近
   q_esti0 << 0, 0;//相对位置估计值初始化（人为设置）
   para_N = 36;//估计器参数
   para_omega = 2*PI/para_N;//估计器参数
   para_Pi(0,0) = cos(para_omega);//估计器参数
   para_Pi(0,1) = (-1.0) * sin(para_omega);//估计器参数
   para_Pi(1,0) = sin(para_omega);//估计器参数
   para_Pi(1,1) = cos(para_omega);//估计器参数
   para_Sigma = Eigen::MatrixXd::Identity(2,2);//单位矩阵
   para_pho0 << 1, 0;//估计器参数初始值
   para_Gamma = para_alpha * Eigen::MatrixXd::Identity(2,2);//估计器增益矩阵

   /*****************************订阅及发布话题*********************************/
   height_sub = n_.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 1000, &uwb_relative_localization::height_callback, this);//激光测高  ~10hz
   distance_sub = n_.subscribe<uwb::uwbrangemsg>("uwb_kf_range", 100,&uwb_relative_localization::distance_callback, this);//UWB测距 ~33hz
   flow_sub = n_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, &uwb_relative_localization::vel_callback,this);//光流计算位移  ENU ~30hz
   local_pos_sub = n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &uwb_relative_localization::local_pos_cb,this);//订阅本地位置信息  ENU坐标系  ~30hz
   relative_pub = n_.advertise<relative_localization::relative_position>("/relative_position/uwb/pos_result", 10);//pub相对位置  ~33hz
   rel_pub_timer = n_.createTimer(ros::Duration(0.1), &uwb_relative_localization::timerCallback,this);//10HZ

   /*****************************init*********************************/
   init=true;
   high_sensor_value = -5;//cm 激光测高高度参数初始化
   rel_msg_count = 0;
}      

void uwb_relative_localization::height_callback(const sensor_msgs::Range::ConstPtr& msg)
{
      high_sensor_value = msg->range;//单位：m 激光测距参数，未矫正距离
      uav_height = high_sensor_value-distance_sensor_range;//uav底部到地面的距离
      pos.z_pos = uav_height;//uav底部到地面的距离
      pos.lidar_value = high_sensor_value;//单位：m 激光测距参数，未矫正距离
      /*
      static double lidar_delta_time_begin = ros::Time::now().toSec();
      static double lidar_delta_time_end   = ros::Time::now().toSec();
      static double lidar_delta_t = 0.1;
      static int lidar_delta_t_count = 0;
      
      ////////////TODO:check distance_sensor_range
      high_sensor_value = msg->range;//m 激光测距参数，未矫正距离
      uav_height = high_sensor_value-distance_sensor_range;//uav底部到地面的距离
      //ROS_INFO("high_sensor_value_in:%f",high_sensor_value);
      //ROS_INFO("uav_height_in:%f",uav_height);
      	
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
      lidar_range = rangekf.filter(Eigen::MatrixXd::Identity(1,1)*(msg->range*100),init,lidar_delta_t,0);//调用卡尔曼滤波
      rangekf.showState();  
	   pos.z_pos = uav_height;//uav底部到地面的距离
      pos.lidar_value = high_sensor_value;//cm 激光测距参数，未矫正距离
      //ROS_INFO("high_sensor_value_out:%f",pos.lidar_value);
      //ROS_INFO("uav_height_out:%f",pos.z_pos);
      */

}


void uwb_relative_localization::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//订阅光流信息（已融合进px4，从mavros中直接获取position），用于计算位移
{
  if(use_pos){
   //无人机实时高度
  //UAV_z = msg->pose.position.z;//单位：m
  //无人机坐标
  UAV_x = msg->pose.position.x;
  UAV_y = msg->pose.position.y;
   //ROS_INFO("vel_x:%f,vel_y:%f",vel_x,vel_y);
   if(init){
      flow_Phi(0,0) = 0;
      flow_Phi(1,0) = 0;//ENU
      UAV_x_old = UAV_x;
      UAV_y_old = UAV_y;
      init = false;
   }
   else{
      flow_Phi(0,0) += UAV_x - UAV_x_old;
      flow_Phi(1,0) += UAV_y - UAV_y_old;//ENU
      //ROS_INFO("use pos; flow_Phi:%f,%f",flow_Phi(0,0),flow_Phi(1,0)); 
      UAV_x_old = UAV_x;
      UAV_y_old = UAV_y;     
   }
  } 
}

void uwb_relative_localization::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)//订阅光流信息（已融合进px4，从mavros中直接获取速度），用于计算位移
{
   if(use_vel){
   vel_x = msg->twist.linear.x;//ENU 
   vel_y = msg->twist.linear.y;
   //ROS_INFO("vel_x:%f,vel_y:%f",vel_x,vel_y);
   if(init){
      flow_delta_time_begin = ros::Time::now().toSec();
      flow_delta_time_end   = ros::Time::now().toSec();
      flow_delta_t = 0.1;
      init = false;
   }
   else{
      flow_delta_time_end   = ros::Time::now().toSec();
      flow_delta_t = flow_delta_time_end-flow_delta_time_begin;//单位：秒
      flow_delta_time_begin = ros::Time::now().toSec();
      if(flow_delta_t>1)flow_delta_t=1;
      //ROS_INFO("flow_delta_t:%f",flow_delta_t);
      //flow_Phi(0,0) += vel_x * flow_delta_t;
      //flow_Phi(1,0) += vel_y * flow_delta_t;//ENU
      flow_Phi(0,0) += vel_x * flow_delta_t;
      flow_Phi(1,0) += vel_y * flow_delta_t;//ENU
      //ROS_INFO("use vel; flow_Phi:%f,%f",flow_Phi(0,0),flow_Phi(1,0));      
   }
  } 
}


void uwb_relative_localization::distance_callback(const uwb::uwbrangemsg::ConstPtr& msg)
{
   if(is_first_step) {      
      d_old = -1;//初始化
      //r_old = -1;
      r_now = msg->dis0/100;//三维空间直线距离 单位：m
      d_now = sqrt( (r_now*r_now) - ((high_sensor_value-target_height)*(high_sensor_value-target_height)) );//单位：m
      para_pho = para_pho0;
      para_sigma = para_Sigma * para_pho;
      q_esti = q_esti0;
      is_first_step = false;
   }
   else{
      d_old = d_now;
      //r_old = r_now;
      r_now = msg->dis0/100;//三维空间直线距离 单位：m
      d_now = sqrt( (r_now*r_now) - ((high_sensor_value-target_height)*(high_sensor_value-target_height)) );//二维平面直线距离
      pos.uwb_distance_2dimension = d_now;//单位：m
      pos.uwb_distance_3dimension = r_now;
   }        
}


void uwb_relative_localization::timerCallback(const ros::TimerEvent& event)
{
   //flow_Phi_2_norm_2 = (flow_Phi(0,0)*flow_Phi(0,0)) + (flow_Phi(1,0)*flow_Phi(1,0));
   //flow_Phi_2_norm_2 = flow_Phi.squaredNorm();
   para_zeta = (0.5) * ( (d_now*d_now) - (d_old*d_old) - flow_Phi.squaredNorm() );
   para_epsilon = para_zeta - ( flow_Phi.transpose() * q_esti );
   q_esti = q_esti + flow_Phi + ( para_Gamma * flow_Phi * para_epsilon );//ENU
   //q_esti_2_norm = sqrt( (q_esti(0,0)*q_esti(0,0)) + (q_esti(1,0)*q_esti(1,0)));
   //q_esti_2_norm = q_esti.norm();
   //if(r_now>q_esti_2_norm){
   //   q_esti = q_esti;//实际输出速度
   //}
   //else{
   //   q_esti = r_now * q_esti / q_esti_2_norm;//实际输出速度 ENU
   //}
   //ROS_INFO("q_esti:%f,%f",q_esti(0,0),q_esti(1,0));
   //更新参数
   ROS_INFO("flow_Phi:%f,%f",flow_Phi(0,0),flow_Phi(1,0));
   flow_Phi(0,0) = 0.0;
   flow_Phi(1,0) = 0.0;//ENU
   para_pho = para_Pi * para_pho;
   para_sigma = para_Sigma * para_pho;
   pos.x_pos_relative = q_esti(0,0);
   pos.y_pos_relative = q_esti(1,0);
   pos.sigma1 = para_sigma(0,0);
   pos.sigma2 = para_sigma(1,0);
   //ROS_INFO("high_sensor_value_out:%f",pos.lidar_value);
   //ROS_INFO("uav_height_out:%f",pos.z_pos);
   pos.header.stamp = ros::Time::now();
   pos.header.seq = rel_msg_count;
   relative_pub.publish(pos);
   rel_msg_count++;
   ROS_INFO("Sending relative position");
   ROS_INFO("Position: x_pos_relative:%f y_pos_relative:%f", pos.x_pos_relative, pos.y_pos_relative); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relative_localization2D");
  uwb_relative_localization rel_pos2D;
  ros::spin();
  /*ros::Rate r(10);//设置发布频率 10HZ
  while (ros::ok()) {
   // read
   ros::spinOnce();  
   r.sleep();//通过睡眠度过一个循环中剩下的时间，来达到该设定频率
  }*/
}

