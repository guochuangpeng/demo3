#ifndef VISUAL_ESTI_H
#define VISUAL_ESTI_H

#include <ros/ros.h>

#include <math.h>
#include <iostream>

#include<opencv2/imgproc/imgproc.hpp>///
#include<opencv2/core/core.hpp>///
#include<opencv2/highgui/highgui.hpp>///

#include <Eigen/Dense>//使用Eigen中的Matrix
#include <opencv2/core/eigen.hpp>

#include "relative_localization/relative_position.h"//订阅距离高度信息
#include "apriltag_ros/AprilTagDetection.h"//订阅视觉计算相对位置信息
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "visual_estimation/visual_rel_pos.h"//发布fused相对位置



class visual_rel_fuse
{
    private: 
        ros::NodeHandle n_;
        ros::Publisher relative_pub;//relative position pub 发布相对位置
        ros::Subscriber distance_height_sub;//订阅与目标的直线距离以及高度信息
        ros::Subscriber relative_sub;//订阅apriltag得到的相对位置

        visual_estimation::visual_rel_pos pos;//用于传递位置信息
    
        //input
        double r_now;//stright distance//三维空间直线距离  来源：uwb
        double uav_height;//the height of uav from lidar //uav底部到地面的距离    
	    double distance_sensor_range;//传感器安装位置距离无人机最下层的高度
	    double high_sensor_value;//cm 激光测距参数，未矫正距离
        Eigen::Matrix<double,3,1> visual_rel;//视觉估计相对位置  来源：apriltag

        //output
        Eigen::Matrix<double,3,1> fused_rel;//estimated relative position//相对位置估计值
        
        

        
    public:
    ///////变量要不要static？？？
        //default conditions
        double target_height;//the height between target and ground 目标距离地面的高度
        bool is_first_step;//init 是否第一步

        visual_rel_fuse();

        void rel_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
        void sensor_callback(const relative_localization::relative_position::ConstPtr& msg);
};

#endif 
