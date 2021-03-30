#ifndef RELATIVE_LOCALIZATION3D_H
#define RELATIVE_LOCALIZATION3D_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <math.h>
#include <iostream>

#include<opencv2/imgproc/imgproc.hpp>///
#include<opencv2/core/core.hpp>///
#include<opencv2/highgui/highgui.hpp>///

#include <Eigen/Dense>//使用Eigen中的Matrix
#include <opencv2/core/eigen.hpp>

#include "uwb/uwbrangemsg.h"
#include "uwb/kf.h"
#include "relative_localization/relative_position.h"//发布相对位置msg

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <sensor_msgs/Range.h>

//#define DIMENSION_NUM 3//or 2

//相比2D版，参数变化如下
// 2D           =>    3D
// C                  para_alpha
// para_alpha         para_gamma
// para_Gamma         para_gamma
///////para_Gamma = para_alpha * I

class uwb_relative_localization
{
    private: 
        ros::NodeHandle n_;
        ros::Publisher relative_pub;//relative position pub 发布相对位置
        ros::Subscriber distance_sub;//uwb distance 订阅与目标的直线距离
        ros::Subscriber flow_sub;//optical flow measurement  订阅光流速度信息（已融合进px4，从mavros中直接获取速度），用于计算位移
        ros::Subscriber local_pos_sub;//订阅本地位置信息
        ros::Subscriber height_sub;//the height of uav from lidar  订阅激光雷达测距信息，用于测高
        ros::Timer rel_pub_timer;
        

        relative_localization::relative_position pos;//用于传递位置信息
    
        /*****************************input*********************************/
        double r_now;//stright distance//三维空间直线距离  来源：uwb
        double r_old;
        double d_now;//distance in this moment//二维平面直线距离
        double d_old;//distance in the previous moment
        Eigen::Matrix<double,3,1> flow_Phi;//displacement data ,measurement from px4flow  来源：px4flow  无人机位移信息
        double flow_Phi_2_norm_2;
        Eigen::Matrix<double,3,1> q_esti0;//相对位置估计值初始化
        
        /*****************************output*********************************/
        Eigen::Matrix<double,3,1> q_esti;//estimated relative position//相对位置估计值
        double q_esti_2_norm;//二范数
        /*****************************calculate parameters*********************************/
        //Eigen::MatrixXf para_Phi(2,1);//displacement of the last movement
        double para_epsilon;//error coefficient 估计器参数
        double para_zeta;//average displacement of last movement 估计器参数
        Eigen::Matrix<double,2,2> para_Pi;//continuous mapping 估计器参数
        //Eigen::Matrix<double,2,2> para_Sigma;//continuous mapping 估计器参数
        Eigen::Matrix<double,3,1> para_sigma;//估计器参数
        Eigen::Matrix<double,2,1> para_pho;//估计器参数
        Eigen::Matrix<double,2,1> para_pho0;//估计器参数初始值
        //Eigen::Matrix<double,3,3> para_Gamma;//估计器参数
        double para_Gamma;//估计器参数
        //Eigen::MatrixXd para_Gamma(2,2);//can't identify this type. why?
        double para_alpha;//估计器参数
        int para_N;//default:36//估计器参数
        double para_omega;////估计器参数
        double r1;
        double r2;

        /*****************************uav parameters*********************************/
        bool lidar_data_enable;//激光雷达测高使能参数
        double uav_height;//the height of uav from lidar //uav底部到地面的距离    单位：m
	    double distance_sensor_range;//单位：m 传感器安装位置距离无人机最下层的高度
	    double high_sensor_value;//单位：m 激光测距参数，未矫正距离
        double high_sensor_value_old;
        double vel_x;//无人机速度x
        double vel_y;//无人机速度y
        
        double UAV_x;
        double UAV_x_old;
        double UAV_y;
        double UAV_y_old;
        
        /*****************************default conditions*********************************/
        double target_height;//the height between target and ground 目标距离地面的高度
        
        /*****************************limited conditions*********************************/
        double d_safe;//safe distance 安全距离 单位：m
        //float u_max;//最大输出速度
        bool is_first_step;
        bool init;//初次接受数据先进行初始化
        bool init_lidar;
        //bool flow_delay_complete;
        
        /*****************************other*********************************/
        double flow_delta_time_begin = ros::Time::now().toSec();
        double flow_delta_time_end   = ros::Time::now().toSec();
        double flow_delta_t = 0.1;
        //double delta_time_begin = ros::Time::now().toSec();
        //double delta_time_end   = ros::Time::now().toSec();
        //double delta_t = 0.1;
        //int delta_t_count = 0;
        
        int rel_msg_count;
        
        bool use_pos;
        bool use_vel;
       
    public:      
        uwb_relative_localization();
        void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);//订阅光流信息（已融合进px4，从mavros中直接获取速度），用于计算位移
        void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);//订阅本地位置信息
        void distance_callback(const uwb::uwbrangemsg::ConstPtr& msg);//订阅UWB测距信息
        void height_callback(const sensor_msgs::Range::ConstPtr& msg);//订阅雷达高度信息
        void timerCallback(const ros::TimerEvent& event);
};

#endif 
