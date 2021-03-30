#ifndef UAV_MOVEMENT_H
#define UAV_MOVEMENT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>//发布速度信息
#include <geometry_msgs/PoseStamped.h>//发布位置信息

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <fstream>

#include<opencv2/imgproc/imgproc.hpp>///
#include<opencv2/core/core.hpp>///
#include<opencv2/highgui/highgui.hpp>///

#include <Eigen/Dense>//使用Eigen中的Matrix
#include <opencv2/core/eigen.hpp>

#include "relative_localization/relative_position.h"//订阅相对位置msg
#include "visual_estimation/visual_rel_pos.h"
//#include "uwb/kf.h"//激光测距信息调用卡尔曼滤波

//#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>

#define PI 3.1415926



class rel_controller
{
    private: 
        ros::NodeHandle nh_;
        ros::Subscriber local_pos_sub;//订阅本地位置信息
        ros::Subscriber uwb_rel_pos_sub;//订阅uwb相对位置
        ros::Subscriber visual_rel_pos_sub;//订阅visual相对位置
        //ros::Subscriber height_sub;//订阅雷达高度信息
        ros::Publisher Hope_vel_pub;//发布输出速度
        ros::Publisher Hope_pos_pub;//发布输出位置

        /**************************msg**************************/
        geometry_msgs::Twist vel_uwb;//uwb定位阶段输出速度
        geometry_msgs::PoseStamped pos_visual;//visaul定位阶段输出位置

        /**************************input**************************/
        //bool lidar_data_enable;//激光雷达测高使能参数
        double uav_height;//the height of uav from lidar  uav底部到地面的距离    
	    double distance_sensor_range;//传感器安装位置距离无人机最下层的高度
	    double high_sensor_value;//cm 激光测距参数，未矫正距离
        double distance;//distance in this moment 与目标的直线距离（二维）
        /***********uwb_phase***********/
        Eigen::Matrix<double,2,1> q_rel_uwb;//relative position相对位置
        Eigen::Matrix<double,2,1> para_sigma;//速度控制器参数
        /***********visual_phase***********/
        Eigen::Matrix<double,2,1> q_rel_visual;//relative position相对位置
        /*********************************************************/
 

        /**************************output**************************/
        /***********uwb_phase***********/
        Eigen::Matrix<double,2,1> u_relative;//计算输出速度
        double u_relative_2_norm;//二范数
        Eigen::Matrix<double,2,1> u_out;//实际输出速度
        /***********visual_phase***********/

        /**********************************************************/
 
        /***********************速度控制器参数***********************/
        //calculate parameters
        double para_beta;//速度控制器参数
        double para_C;//速度控制器参数
        /**********************************************************/

        /**************************UAV参数**************************/
        double UAV_z;//ENU坐标系
        double UAV_x;//ENU坐标系
        double UAV_y;//ENU坐标系
        //无人机机头朝向与正北方向（即ENU中的y轴正方向）的夹角 逆时针夹角为正
        double UAV_rotate;//ENU坐标系 偏航角度，以起飞点为初始位置
        double vel_x;//当前x速度
        double vel_y;//当前y速度
        /**********************************************************/
        

        ros::ServiceClient arming_client;//Service中的 Client节点
        ros::ServiceClient set_mode_client;//更改mavros的模式，比如offboard
        ros::Timer uwb_pub_timer;
        
        bool uav_ready;//无人机准备完毕与否
        bool close_target;//接近目标标志
        bool uwb_phase;//uwb驱动阶段
        bool visual_phase;//visual驱动阶段
        bool detection_success;
        bool should_land;
        double u_max;//maximum velocity最大输出速度
        double highest_height;//最大飞行高度
        double target_height;
        double set_height;//设置飞行高度
        
        bool visual_enable;
        double d_safe;//safe distance 安全距离

    public:
        

        rel_controller();//构造函数
        //void height_callback(const sensor_msgs::Range::ConstPtr& msg)/////pose.pose问题待解决
        void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);//订阅本地位置信息
        void uwb_rel_pos_cb(const relative_localization::relative_position::ConstPtr& msg);//订阅相对位置信息uwb
        void visual_rel_pos_cb(const visual_estimation::visual_rel_pos::ConstPtr& msg);//订阅相对位置信息visual
};

#endif 
