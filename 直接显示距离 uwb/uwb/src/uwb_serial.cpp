#include "serial/serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "ros/ros.h"
#include <iostream>
#include <string>//std::msg
#include <vector>
#include <stdio.h>//sscanf
#include<regex>

std::string com_read_tmp;
std::vector<std::string> dwm_rec_His;
serial::Serial ros_ser;

// 回调函数
void uwbWriteCallback(const std_msgs::String::ConstPtr &msg){
     ROS_INFO_STREAM("Write to serial port" << msg->data);
     ros_ser.write(msg->data);
}

int uwbReadCallback(ros::Publisher &sensor_pub)
{          
    //获取串口数据
    std::string buf;
    //std::regex smatch m;
    std::smatch result;
    buf = ros_ser.read(ros_ser.available());//读取, 4 kinds of input type.
 //   std::regex re12("TAG_ID:  0	");
  ///ROS_INFO_STREAM("Read: " << buf);
   
    //regex  re12(R'(\d+) cm');
    if(std::regex_search(buf, result,std::regex("((\\d+) cm)"))){
                 ROS_INFO_STREAM("len: " << result[2].str());
                 std_msgs::String serial_data;
                        serial_data.data=result[2];
                        ROS_INFO_STREAM("publish:"<<serial_data);
                        sensor_pub.publish(serial_data);
                }
    else{//ROS_INFO_STREAM("Read: " << buf);

    }
   
    //ROS_INFO("recevie size:%zd",buf);//size_t有专用的类型说明符%zd
	//ROS_INFO_STREAM("[1]");
    if(buf.length())
    {
        ////while长度，不断检测完整性，再解码放入数组
        com_read_tmp+=buf;//全局变量，缓冲串口数据
        //eg:mr 0f 000005a4 000004c8 00000436 000003f9 0958 c0 40424042 a0:0\r\n
        //虚拟串口一次攒大量数据再传输而不是实时传输，因此要判断长度循环处理

        int length = com_read_tmp.length();
        int TOF_REPORT_LEN  =(47);//65计算到\r
        std::string df_head="T";//数据帧头
        std::string df_tail="\r\n";//数据帧尾

        //00.保证数据完整
        if((com_read_tmp.find(df_head)<com_read_tmp.rfind(df_tail))&&
            (com_read_tmp.find(df_tail)!=std::string::npos)&&(com_read_tmp.find(df_head)!=std::string::npos))//C++的find函数找到数值的话，就bai输出。否du则，返回string::npos
        {//find_first_of只能按字符而不是字符串搜索
        //find会找到第一个满足的字符串
        	//ROS_INFO_STREAM("[4]");
            while((length >= TOF_REPORT_LEN)&&ros::ok()) //0.获得数据的长度大于协议长度
            {
            	//ROS_INFO_STREAM("[5]");
                //1.保证数据完整//检查一组数据,确保每帧都正常
                if((com_read_tmp.find(df_head)<com_read_tmp.rfind(df_tail))&&
                (com_read_tmp.find(df_tail)!=std::string::npos)&&(com_read_tmp.find(df_head)!=std::string::npos))
                {
                	//ROS_INFO_STREAM("[6]");
                    com_read_tmp.erase(0,com_read_tmp.find(df_head));//完成一次传输移除帧头前的数据，避免传输失败的影响

                    // ROS_INFO_STREAM(com_read_tmp);
                    // ROS_INFO_STREAM(com_read_tmp.length()%65);

                    //2.确保数据完整后，添加进历史列表
                    dwm_rec_His.push_back(com_read_tmp.substr(0,com_read_tmp.find(df_tail)));

                    if(dwm_rec_His.size()>100){
                        dwm_rec_His.erase(dwm_rec_His.begin());//删除最早加入的数据
                    }
                    com_read_tmp.erase(0,com_read_tmp.find(df_tail)+df_tail.length());//删除最早加入的数据(pos,len)~~~~~
                
                    length = com_read_tmp.length();
                    //3.读取历史列表最后最新一组数据.进入解码处理并放入数据列表中
                    std::string tmp=dwm_rec_His.back();//最新一组数据
                    int aID, tID, range[4], lnum, seq, mask;
                    int rangetime;
                    char c, type;
                    //很棒的方法，完成了数据切分并实现hex自动转为int数据
                    int n = sscanf(tmp.data(),"m%c %x %x %x %x %x %x %x %x %c%d:%d", &type, &mask, &range[0], &range[1], &range[2], &range[3], &lnum, &seq, &rangetime, &c, &tID, &aID);
                    //数据流实例：     mc 01 00000443 00000000 00000000 00000000 0190 91 00022f00 a0:0
                    //mc mr 交叉      mr 01 00000443 00000000 00000000 00000000 0190 91 40224022 a0:0
                    int TOF_REPORT_ARGS = (12);
                    aID &= 0x3;//按位与并赋值

                    if(n != TOF_REPORT_ARGS)//不能完整切分
                    {
                        ROS_INFO_STREAM("can't split->"<<tmp);
                        continue;
                    }
                    else
                    {
                    	//ROS_INFO_STREAM("[2]");
        // //           receviedCount++;//接收计数
                    }
                    unsigned int anchor_count = 0,mask_tmp=mask ; // 计数器
                    while (mask_tmp > 0)
                    {
                        if((mask_tmp &1) ==1) // 当前位是1
                        ++anchor_count ; // 计数器加1
                        mask_tmp >>=1 ; // 移位
                    }
                    if(anchor_count<3)
                    {
                        //ROS_ERROR_STREAM("Available anchor cout :"<<anchor_count<<",can't be use to localization.[add judge auto landing code here]\n");
                        // continue; //maybe only show error message is better, vehicle get the wrong positon is better than can't get postion and crash.
                    }
                    //分首次获取设备信息和之后获取测距值
                    if(type=='c')
                    {
                        std_msgs::String serial_data;
                        serial_data.data=tmp;
                        ROS_INFO_STREAM("publish:"<<serial_data);
                        sensor_pub.publish(serial_data);
                    }
                    else if(type=='r')
                    {
                    	//ROS_INFO_STREAM("R");
                        continue;
                    }
                    else if(type=='a')
                    {
                        continue;
                        //ROS_INFO_STREAM("A");
                    }
                }
                else
                {
                	//ROS_INFO_STREAM("[data error]");
                    break;//一组数据的中间数据有误，跳出while继续接收
                }
            }
            
        }
        //ROS_INFO_STREAM("[3]");
        return 0;//传输不完整
    }
    return -1;//未完成一次传输
}


int main (int argc, char** argv){
    ros::init(argc, argv, "uwb_serial");
    ros::NodeHandle nh;
    //订阅主题command
    ros::Subscriber command_sub = nh.subscribe("uwb_serial_write",1000,uwbWriteCallback);
    //发布主题sensor
    ros::Publisher sensor_pub = nh.advertise<std_msgs::String>("uwb_serial_read", 1000);

    try
    {
        ros_ser.setPort("/dev/ttyUSB0");//stm虚拟出口
        //first case :use stm_com to get uwb serial data and send the data to zigbee serial module
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException& e)
    {
        try
        {
            ros_ser.setPort("/dev/uwb_02");//second case: use serial2usb module to receive the uwb serial data.
            ros_ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ros_ser.setTimeout(to);
            ros_ser.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            return -1;
        }
    }
    if(ros_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
        ros_ser.flush();//清空防止历史数据过多
        ROS_INFO_STREAM("flush buffer");
        if(ros_ser.available())
        {
            // ros_ser.read(ros_ser.available());
            // while(ros_ser.available())
            // {
            //     ros_ser.read(ros_ser.available());
            // }
            ROS_INFO_STREAM("clear the buffer");
        }
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        if(ros_ser.available())
        {
            uwbReadCallback(sensor_pub);
            //ROS_INFO_STREAM("not data");
        }
        else
        {
            
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    ros_ser.close();
 }


