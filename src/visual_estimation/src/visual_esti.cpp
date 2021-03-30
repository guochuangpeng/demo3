#include "visual_esti.h"
 
#define PI 3.1415926


visual_rel_fuse::visual_rel_fuse()
{
  /*****************************param*********************************/
  ros::NodeHandle private_nh_("~"); 
  private_nh_.param<double>("target_height", target_height, 0.0);//目标的高度
  
  is_first_step = true;//是否第一步
  

  /*****************************订阅及发布话题*********************************/
  distance_height_sub = n_.subscribe<relative_localization::relative_position>("/relative_position/uwb/pos_result", 10, &visual_rel_fuse::sensor_callback, this);//激光测高
  relative_sub = n_.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, &visual_rel_fuse::rel_callback,this);
  relative_pub = n_.advertise<visual_estimation::visual_rel_pos>("/relative_position/visual/pos_result", 10);//pub相对位置

  high_sensor_value = -5;//cm 激光测高高度参数初始化
}      

void visual_rel_fuse::sensor_callback(const relative_localization::relative_position::ConstPtr& msg)
{
  r_now = msg->uwb_distance_3dimension;//三维空间直线距离
  uav_height = msg->z_pos;//uav底部到地面的距离
  high_sensor_value = msg->lidar_value;//cm 激光测距参数，未矫正距离
}

void visual_rel_fuse::rel_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  visual_rel(0,0) = msg->detections[5].pose.pose.pose.position.x;
  visual_rel(1,0) = msg->detections[5].pose.pose.pose.position.y;
  visual_rel(2,0) = msg->detections[5].pose.pose.pose.position.z;

  fused_rel(0,0) = visual_rel(0,0);
  fused_rel(1,0) = visual_rel(1,0);
  fused_rel(2,0) = visual_rel(2,0);
  pos.x_pos_relative = fused_rel(0,0);
  pos.y_pos_relative = fused_rel(1,0);
  pos.z_pos_relative = fused_rel(2,0);
  relative_pub.publish(pos);
  ROS_INFO("Sending relative position");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_esti");
  visual_rel_fuse rel_pos;

  ros::Rate r(20);  // slower than detection
  while (ros::ok())
  {
     // read
     ros::spinOnce();  

     r.sleep();
  }
}
