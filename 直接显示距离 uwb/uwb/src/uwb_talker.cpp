#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uwb/uwbrangemsg.h"
#include "uwb/kf.h"
#include <string>//string
#include <stdio.h>//sscanf
#include <stdlib.h>//rand
#include "uwb/uwb_talker.h"
#include <vector>
#include <numeric>
#include <iterator>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Range.h>

using namespace std;


kalmanFilter kf0(1,1);//uwb ranging data//对测距信息进行卡尔曼滤波
kalmanFilter rangekf(1,1);//lidar data//对激光雷达测高进行卡尔曼滤波

Eigen::MatrixXd uwb_measure(1,1);//uwb ranging data//uwb的测距信息存储，滤波后的数据
vector<double> uwb_raw_dis;//raw data//uwb的测距信息的原始数据
vector<double> uwb_kf_dis;//kf-ed data//uwb的测距信息经过卡尔曼滤波的滤波数据
 
class UwbTalker  
{  
public:  
    UwbTalker()  
    {
        ros::NodeHandle private_nh_("~"); 
        private_nh_.param("timer_pus_Hz", timer_pus_Hz, 40);//???

        uwb_raw_range_pub_=n_.advertise<uwb::uwbrangemsg>("uwb_raw_range", 100);//pub raw data//传输uwb原始测距数据的msg
        uwb_kf_range_pub_=n_.advertise<uwb::uwbrangemsg>("uwb_kf_range", 100);//pub kf-ed data//传输uwb经过卡尔曼滤波的滤波数据的msg                                                                                                                                         

        sub_ = n_.subscribe("uwb_serial_read", 100, &UwbTalker::callback, this);//订阅串口信息 自身的callback

        acc2pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("acc2pose/matlab", 100);//发布记录采集数据的时间，用于matlab分析   

        srand(1);// rand seed
        uwb_msg_count = 0;
        
        init=true;  //初次接受数据先进行初始化

        delta_time_begin = ros::Time::now().toSec();
        delta_time_end   = ros::Time::now().toSec();
        current_time=ros::Time::now().toSec();
        delta_t = 0.032;//
        delta_t_count = 0;//
        last_not_outlier = 0;
        //memset(last_not_outlier,0,sizeof(last_not_outlier));//数组初始化
    }    
  
    void callback(const std_msgs::String::ConstPtr& msg)  
    {  
        //定义数距流包含的参数
        int aID, tID, range[4] = {0}, lnum, seq, mask;//range[]分别为按顺序到四个anchor的测量信息，这里只用一个
        int rangetime;
        char c, type;
        string tmp;
             
        tmp=msg->data.c_str();
        //int n = sscanf(tmp.data()," %x ",  &range[0]);
        //mc 01 0000042c 00000000 00000000 00000000 0191 92 00014022 a0:0   lnum和rangetime会随时间累计 
        //mr 01 0000042c 00000000 00000000 00000000 0191 92 00014022 a0:0
        formatlab.header.stamp=ros::Time::now();//记录采集数据的时间，用于matlab分析
        
        // if(!((0x1 << 0) & mask)) //we have a valid range  判断测距数距流信息是否正确   0x1 << k  :k左移一位
        // {
        //     range[0] = kf_mean*10;//没收到数据时填充kf后的均值，kf_mean单位为cm，但range这里为mm
        //     //ROS_ERROR("Lost:%d",k);
        // }
        // else/////////////////////////////////////////////least squart
        // {
        //     range[0] = 0.9899 * range[0] -517;//单位mm   UWB校准 线性拟合
        // }
        range[0]= stoi(msg->data);
        /**********原始的测距值数据处理之前****************/
        ROS_INFO_STREAM("原始" << tmp);
        ROS_INFO_STREAM("range" << range[0]);
        range[0]= range[0]*10;
        uwb_raw_msg.dis0 = range[0]*1.0/10;//把单位mm转换为cm
        if(1)//有效数据流
        {
            /*************这样子求dt好吗？   ***********/
            delta_t_count++;
            if(delta_t_count>=100)//50hz,~2s
            {
                delta_time_end   = ros::Time::now().toSec();//单位：秒
                delta_t = (delta_time_end-delta_time_begin)/100;
                delta_time_begin = ros::Time::now().toSec();
                delta_t_count = 0;
                if(delta_t>1)delta_t=1;
                //ROS_INFO("%f",delta_t);0.018
            }
            
            // record raw & kf data 记录十个数据
            
            uwb_raw_dis.push_back(range[0]*1.0/10);  //range为没有处理过的原始数据cm
            
            //uwb_measure.block(k,0,1,1)<<kf0.filter(Eigen::MatrixXd::Identity(1,1)*range[0]*1.0/10,init,0,delta_t);
            uwb_kf_dis.push_back(uwb_measure(0,0));//uwb_measure为kf滤波后的数据cm
            if(uwb_raw_dis.size()>10)//vector超出十个数距删除最前面的数据
            {
                uwb_raw_dis.erase(uwb_raw_dis.begin());//删除最前面的数据
                uwb_kf_dis.erase(uwb_kf_dis.begin());
            }
            
            /*******************求均值和标准差*************************/
            
            double raw_sum = accumulate(uwb_raw_dis.begin(),uwb_raw_dis.end(),0.0);//accmulate求和
            double kf_sum = accumulate(uwb_kf_dis.begin(),uwb_kf_dis.end(),0.0);
            raw_mean = raw_sum/uwb_raw_dis.size();//均值 单位cm
            kf_mean = kf_sum/uwb_kf_dis.size();

            double raw_accum = 0.0;//定义方差参数
            for(int i=0;i<uwb_raw_dis.size();i++)
            {
                raw_accum += (uwb_raw_dis[i]-raw_mean)*(uwb_raw_dis[i]-raw_mean);//求方差
                //ROS_INFO("dis:%f",uwb_raw_dis[k][i]);
            }

            double kf_accum = 0.0;
            for(int i=0;i<uwb_kf_dis.size();i++)
            {
                kf_accum += (uwb_kf_dis[i]-kf_mean)*(uwb_kf_dis[i]-kf_mean);
            }
            raw_std = sqrt(raw_accum/(uwb_raw_dis.size()-1));//标准差
            kf_std = sqrt(kf_accum/(uwb_kf_dis.size()-1));
        

            /******************异常值处理*********************************/
            double dis_tmp=0;
            
            if(abs(range[0]*0.1-raw_mean)>2.5*raw_std)//log记录输出的是没去异常值前的测距值而不是这里的输出
            //range单位为mm，单位不统一，因此才*0.1，化为cm
            {
                if (range[0]*0.1-raw_mean>0)
                {
                    dis_tmp = last_not_outlier + 0.15*raw_std*10;//单位mm
                }
                else
                {
                    dis_tmp = last_not_outlier - 0.15*raw_std*10;
                }
            }
            else//无异常值
            {
                dis_tmp = range[0];//单位mm
                last_not_outlier = range[0];//单位mm
            }
            //cout<<"[range1]: "<<k<<" :"<<range[k]<<" :"<<abs(range[k]-raw_mean[k])<<" :"<<raw_mean[k]<<" :"<<last_not_outlier[k]<<endl;
           

            /**************卡尔曼滤波*******************/
            Eigen::MatrixXd dis0(2,1);

            //对距离信息进行卡尔曼滤波
            dis0 = kf0.filter(Eigen::MatrixXd::Identity(1,1)*dis_tmp*1.0/10,init,delta_t,raw_std);

            uwb_measure(0,0) = dis0(0,0);
                        
            if(init){init=false;}
          
	        /*******经过异常值处理和当没接收到数据时填充kf后的均值处理***********/
            uwb_raw_msg.dis0_mean = raw_mean;
            uwb_raw_msg.dis0_std = raw_std;
            uwb_raw_range_pub_.publish(uwb_raw_msg);//发布uwbrangemsg

            /***********kf之后的测距值、均值、标准差*****************/
            uwb_kf_msg.dis0 = uwb_measure(0,0);
            uwb_kf_msg.dis0_mean = kf_mean;
            uwb_kf_msg.dis0_std = kf_std;
            uwb_kf_range_pub_.publish(uwb_kf_msg);//发布uwbrangemsg
            
            /**********原始数据的时间**************************/
            
            formatlab.header.seq = uwb_msg_count;
            acc2pose_pub_.publish(formatlab);//记录采集数据的时间，用于matlab分析
            uwb_msg_count++;
        }
    }

/***********************************************************/
private:  
	ros::NodeHandle n_;//ros句柄
	ros::Publisher uwb_raw_range_pub_;//发布uwb原始测距数据的msg
	ros::Publisher uwb_kf_range_pub_;//发布uwb经过卡尔曼滤波的滤波数据的msg

	ros::Subscriber sub_;//订阅串口信息  

	ros::Publisher acc2pose_pub_;//记录采集数据的时间，用于matlab分析
	geometry_msgs::PoseStamped formatlab;//记录采集数据的时间，用于matlab分析

	uwb::uwbrangemsg uwb_raw_msg;//传输uwb原始测距数据的msg
	uwb::uwbrangemsg uwb_kf_msg;//传输uwb经过卡尔曼滤波的滤波数据的msg

    bool init;//初次接受数据先进行初始化

    double delta_time_begin;
    double delta_time_end;
    double delta_t;
    double current_time;
    int delta_t_count;

    double raw_std;
    double kf_std;
    double raw_mean;
    double kf_mean;
    double last_not_outlier;//存储非离群值 上一组正常的数据
    
	int timer_pus_Hz;
	unsigned int uwb_msg_count;

};//End of class UwbTalker  


int main(int argc, char **argv)
{   ROS_INFO(",delta_t");
    ros::init(argc, argv, "uwb_talker");//传入main参数数量和参数字符数组并为包命名
    UwbTalker talker;

    ros::spin();

    return 0;
}
