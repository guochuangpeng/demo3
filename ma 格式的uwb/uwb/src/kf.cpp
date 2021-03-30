/**
  ******************************************************************************
  * @file     kf.cpp
  * @author   JIAWEN
  * @date     
  * @brief    
  * @log	  首次完成
  ******************************************************************************
  */
#include "uwb/kf.h"
#include "ros/ros.h"

/*
 * 函数名： EKF
 * 描述 ：  predict and update the sensor data with nonlinear Gaussian distribution.
 * 输入 ：  
 * 输出 ：
 * 调用 ：  外部调用
 */

const Eigen::MatrixXd kalmanFilter::filter(Eigen::MatrixXd disMatrix,//cm
                   bool init,
                   //int mask,
                   double delta,
                   double std)
{
    sys_init = init;
    sensor_data = disMatrix;
    //sensor_mask = mask;
    delta_t = delta;
    if(sys_init)
    {
        this->kfInit();
    }
    else
    {
        this->kfPredict();
        if(outlier_detetor_threshold != -1)
        {
            this->kfUpdateEvaluate(std);
	    }
        this->kfUpdate();
    }
    return X.block(0,0,dimension*2,1);
}


//这里加一个acc
void kalmanFilter::kfInit()
{  
    X<<sensor_data(0,0),0;  //TODO:for m sensor!!
    P<<Eigen::MatrixXd::Identity(dimension*2,dimension*2)*100;//初始化为大的主对角线矩阵，大的置信空间，如最大范围的2倍，让它自行迭代更新调整
    R = Eigen::MatrixXd::Identity(sensor_num,sensor_num);
    R = R * 45;//85.5cm
}

//F,P,Q
void kalmanFilter::kfPredict()
{
    //predict
    A<<1,delta_t,
       0,delta_t;  //TODO:m,n
    //a_x=500,a_y=500,a_z=500;//除了max_a,其余参数全部需要每次更新   
    acc = 700; // cm/s*s
    
    Q<<0.25*pow(delta_t,4)*pow(acc,2),    0.5*pow(delta_t,3)*pow(acc,2),//TODO:m,n
       0.5*pow(delta_t,3)*pow(acc,2),     pow(delta_t,2)*pow(acc,2);
    
    X_ = A*X+U;
    P_ = A*P*A.transpose()+Q;
}

//H,R
void kalmanFilter::kfUpdateEvaluate(double tmp)
{
    for(i=0;i<sensor_num;i++)
    {
        update_evaluate(i,0) = tmp;
    }
}

const Eigen::MatrixXd kalmanFilter::kfUpdate()
{
    H << 1,0;//TODO:m,n
    K = P_*H.transpose()*(H*P_*H.transpose()+R).inverse();
    d_ = H*X_;
    
    // update
    x_tmp.setZero(dimension*2,1);
    p_tmp.setZero(dimension*2,dimension*2);  // init

    for(i=0;i<sensor_num;i++)
    { 
        if(sys_init)
        {
            x_tmp += K.col(i)*(sensor_data(i,0)-d_(i,0));
            p_tmp += K.col(i)*H.row(i);
        }
        else
        {
            if(outlier_detetor_threshold != -1)
            {
                if(update_evaluate(i,0)<outlier_detetor_threshold)//设定阈值,原：2,mask是模块传输 (TODO:ADD KF MEAN)//&&((0x1 << i) & mask)
                {
                    x_tmp += K.col(i)*(sensor_data(i,0)-d_(i,0));
                    p_tmp += K.col(i)*H.row(i);
                }
                else
                {
                //异常传感器值不添加,以预测值作为当前时刻状态值
                //ROS_ERROR("##########outlier:%f###",update_evaluate(i,0));
                }
            }
            else
            {
                //if(((0x1 << i) & sensor_mask)) //we have a valid range ADD KF MEAN
                {
                    x_tmp += K.col(i)*(sensor_data(i,0)-d_(i,0));
                    p_tmp += K.col(i)*H.row(i);
                }
            }
        }
    }

    X << X_+x_tmp;
    P << (Eigen::MatrixXd::Identity(dimension*2,dimension*2)-p_tmp)*P_;
    
/*
    //标准形式
    K = P_*H.transpose()*(H*P_*H.transpose()+R).inverse();
    X = X_+K*(dis_Matrix.block(0,0,4,1)-d_);
    P = (Eigen::MatrixXd::Identity(6,6)-K*H)*P_;
*/
    return X;
}

void kalmanFilter::setThreshold(double threshold)
{
    outlier_detetor_threshold = threshold;
}

void kalmanFilter::setSingleSensorR(double r)
{
    R = Eigen::MatrixXd::Identity(sensor_num,sensor_num);
    R = R * r;//10cm
}

void kalmanFilter::showState(void)
{
/*
    std::cout<< "\nX:" << X<< std::endl;
    std::cout<< "\nP:" << P<< std::endl;
    
    std::cout<< "\nQ:" << Q<< std::endl;
    std::cout<< "\ny:" << sensor_data-d_<< std::endl;
    */
}



/////don't delete... will use it later

// const Eigen::MatrixXd ekf()
// {
//     this->ekfInit(); // 根据是否使用高度传感器选择状态数
//     this->kfPredict();
//     if(outlier_detetor_threshold != -1)
//     {
//         update_trehold = this->kfUpdateEvaluate();
//     }
//     this->kfUpdate();
// }



// const Eigen::MatrixXd ekfInit(const int m,
//             //Eigen::MatrixXd state_Matrix,
//             Eigen::MatrixXd disMatrix,//cm
//                     bool init,
//                     int mask,
//                     double delta_t,
//                     bool use_other_sensor_high,
//                     double high_sensor_value//cm
//             //Eigen::MatrixXd anchor_Matrix
//             )
// {
//     //输入：m个传感器，n个状态值,维度
//     //tag位置矩阵与速度矩阵（状态），anchor位置矩阵
//     //tag与anchor距离矩阵（观测）
//     //输出：tag位置矩阵
//     //固定参数：加速度协方差矩阵
    
//     int n;
//     double uav_high;
//     if (use_other_sensor_high)
//     {
//         n = 2; // only use uwb to get XY position
//         uav_high = high_sensor_value;  // sensor update frequence higher than UWB, filter should be taken elsewhere
//     }
//     else
//     {
//         n = 3;
//     }


//     //测量噪声方差
//     R = Eigen::MatrixXd::Identity(m,m);
//     for(int i = 0;i<m;i++)//参数是校正前的参数，输入是校正后，会有一些偏差，应该相差不大
//     {
//         R(i,i) = R(i,i)*10;//10cm
//     }

//     if(init)
//     {
        
//         lastDisMatrix.setZero(m,1);

//         //读取并设定基站坐标
//         //使用GN获得初始位置（静止）供ekf（运动）作为初始值

//         std::vector<float> tmpAnchorparam;
//         if (ros::param::has("~anchor0")&&ros::param::has("~anchor1")&&ros::param::has("~anchor2")&&ros::param::has("~anchor3"))
//         {
//             if(ros::param::get("~anchor0",tmpAnchorparam))
//                 for(unsigned i=0; i < tmpAnchorparam.size(); i++) 
//                     anchor_Matrix(0,i)=tmpAnchorparam[i];
//             else ROS_ERROR("anchor0 param set error");

//             if(ros::param::get("~anchor1",tmpAnchorparam))
//                 for(unsigned i = 0; i < tmpAnchorparam.size(); i++) 
//                     anchor_Matrix(1,i) = tmpAnchorparam[i];
//             else ROS_ERROR("anchor1 param set error");

//             if(ros::param::get("~anchor2",tmpAnchorparam))
//                 for(unsigned i = 0; i < tmpAnchorparam.size(); i++) 
//                     anchor_Matrix(2,i) = tmpAnchorparam[i];
//             else ROS_ERROR("anchor2 param set error");

//             if(ros::param::get("~anchor3",tmpAnchorparam))
//                 for(unsigned i = 0; i < tmpAnchorparam.size(); i++) 
//                     anchor_Matrix(3,i) = tmpAnchorparam[i];
//             else ROS_ERROR("anchor3 param set error");
//         }
//         else
//         {
//             ROS_FATAL("can't find all 4 anchor param");
//         }

//         x_max = 0;y_max = 0;z_max = 0;
//         for(int i = 0; i<m;i++)
//         {
//             for(int j = 0; j<m;j++)
//             {
//                 if((x_max)<(anchor_Matrix(i,0)-anchor_Matrix(j,0)))
//                     x_max = (anchor_Matrix(i,0)-anchor_Matrix(j,0));
//                 if((y_max)<abs(anchor_Matrix(i,1)-anchor_Matrix(j,1)))
//                     y_max = (anchor_Matrix(i,1)-anchor_Matrix(j,1));
//                 if((z_max)<(anchor_Matrix(i,2)-anchor_Matrix(j,2)))
//                     z_max = (anchor_Matrix(i,2)-anchor_Matrix(j,2));
//             }
//         }

//         if(n==3)
//         {
//             state_Matrix<<x_max/2,y_max/2,-50,
//                            1,1,1;//参数迭代初值
//         }
//         else
//         {
//             state_Matrix<<x_max/2,y_max/2,
//                            1,1;//参数迭代初值
//         }

        

//         target_axes = calGaussNewton(m,n,GN_threshold,GN_iterations,anchor_Matrix,disMatrix,state_Matrix.block(0,0,n,1));
//         state_Matrix.block(0,0,n,1) = target_axes;

//         X<<state_Matrix;
//     }//init end


//     if(n==3)
//     {
//         A<<1,0,0,delta_t,0,0,
//            0,1,0,0,delta_t,0,
//            0,0,1,0,0,delta_t,
//            //0,0,1,0,0,0//不是真的上一时刻的z进行预测而是直接使用高精度传感器值（要滤波吗？）
//            0,0,0,1,0,0,
//            0,0,0,0,1,0,
//            0,0,0,0,0,1;
           
//         P<<Eigen::MatrixXd::Identity(n*2,n*2)*1000;//初始化为大的主对角线矩阵，大的置信空间，如最大范围的2倍，让它自行迭代更新调整

//         Q<<     0.25*pow(delta_t,4)*pow(a_x,2),0,0,      0.5*pow(delta_t,3)*pow(a_x,2), 0,0,
//                 0,0.25*pow(delta_t,4)*pow(a_y,2),0,      0, 0.5*pow(delta_t,3)*pow(a_y,2),0,
//                 //0,0,0,                                   0,0,0,
//                 0,0,0.25*pow(delta_t,4)*pow(a_z,2),      0, 0,0.5*pow(delta_t,3)*pow(a_x,2),
//                 0.5*pow(delta_t,3)*pow(a_x,2),0,0,       pow(delta_t,2)*pow(a_x,2), 0,0,
//                 0,0.5*pow(delta_t,3)*pow(a_y,2),0,       0,pow(delta_t,2)*pow(a_y,2),0,
//                 //0,0,0,                                   0,0,0;
//                 0,0,0.5*pow(delta_t,3)*pow(a_z,2),       0, 0,pow(delta_t,2)*pow(a_z,2);
//     }
//     else
//     {
//         A<<1,0,delta_t,     0,
//            0,1,  0,     delta_t,
//            0,0,  1,         0,
//            0,0,  0,         1;

//         P<<Eigen::MatrixXd::Identity(n*2,n*2)*1000;//初始化为大的主对角线矩阵，大的置信空间，如最大范围的2倍，让它自行迭代更新调整

//         Q<<     0.25*pow(delta_t,4)*pow(a_x,2),0,      0.5*pow(delta_t,3)*pow(a_x,2), 0,
//                 0,0.25*pow(delta_t,4)*pow(a_y,2),      0, 0.5*pow(delta_t,3)*pow(a_y,2),
//                 0.5*pow(delta_t,3)*pow(a_x,2),0,       pow(delta_t,2)*pow(a_x,2), 0,
//                 0,0.5*pow(delta_t,3)*pow(a_y,2),       0,pow(delta_t,2)*pow(a_y,2);
//     }
// }

/**********************JIAWEN************END OF FILE********************************************/
