#ifndef KF_H
#define KF_H

#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>


class kalmanFilter
{
    public:
    kalmanFilter(int m,int n)
    {
        /***************ekf参数***********************/
        // double delta_t=0.025;//更新频率约56Hz,25
        // a_x=500,a_y=500,a_z=500;//除了max_a,其余参数全部需要每次更新   (cm/s)*delta_t
        v_0 = 0, v_1 = 0, acc = 0;
        i = 0;
        delta_t = 0.025;  // 更新频率约56Hz,25
        outlier_detetor_threshold = -1;
        sensor_num = m;
        //sensor_mask = 0;
        dimension = n;
	    sys_init = true;

        ////////////参数矩阵需记忆迭代更新
        X.resize(dimension*2,1);//状态值
        P.resize(dimension*2,dimension*2);// 状态的协方差矩阵）（相关矩阵），m个特征
        U.resize(dimension*2,1);//
        U.setZero(dimension*2,1);
        ////////////

        A.resize(dimension*2,dimension*2);// 状态转移矩阵
        Q.resize(dimension*2,dimension*2);// 状态转移矩阵的协方差矩阵,模型预测中带来噪声，或是说模型预测后的新的高斯分布，用加法直接体现出来,测量噪声
        d_.resize(sensor_num,1);// 预测距离矩阵
        M_dis.resize(sensor_num,1);// Mahalanobis距离矩阵 马氏距离

        //     static Eigen::Matrix<double, m, 6> H;
        r.resize(sensor_num,1);// 二维距离矩阵
        H.resize(sensor_num,dimension*2);// 观测矩阵，传感器参数转换
        R.resize(sensor_num,sensor_num);// 观测的噪声方差
        ///////////
        X_.resize(dimension*2,1);
        P_.resize(dimension*2,dimension*2);
        K.resize(dimension*2,sensor_num);

        ///////////
        x_tmp.resize(dimension*2,1);
        p_tmp.resize(dimension*2,dimension*2);
        /////////////////
        sensor_data.resize(sensor_num,1);
        update_evaluate.resize(sensor_num,1);
        //lastDisMatrix(sensor_num,1);// 用于判断丢包率，两次数值相同则丢包

        //测量噪声方差
        R = Eigen::MatrixXd::Identity(sensor_num,sensor_num);
        R = R*45;// 10cm
        // for(int i = 0;i<sensor_num;i++)// 参数是校正前的参数，输入是校正后，会有一些偏差，应该相差不大
        // {
        //     R(i,i) = R(i,i)*10;// 10cm
        // }
        /***************ekf参数-end***********************/
    }
    const Eigen::MatrixXd filter(Eigen::MatrixXd,// cm
                   bool init,
                   //int mask,
                   double delta,
                   double std);
    void setThreshold(double threshold);
    void setSingleSensorR(double r);
    
    void showState(void);
    
    private:
        // const Eigen::MatrixXd cal();
        void kfInit();
        void kfPredict();
        void kfUpdateEvaluate(double tmp);                             
        const Eigen::MatrixXd kfUpdate();


        int sensor_num;// m
        //int sensor_mask;
        int state_num;// n, 原本的n/2 TODO:用getshape自动
        int dimension;// 原本的n，在空间中有几维特征
        double outlier_detetor_threshold;
        bool sys_init;
        Eigen::MatrixXd sensor_data;
        Eigen::MatrixXd update_evaluate;

        /***************ekf参数***********************/
        //  double delta_t=0.025;// 更新频率约56Hz,25
        // double a_x,a_y,a_z;// 除了max_a,其余参数全部需要每次更新   cm/s
        double v_0,v_1,acc;
        int i;
        // double update_threshold;// 判断测量值异常值的阈值
	    double delta_t;
        ////////////参数矩阵需记忆迭代更新
        Eigen::MatrixXd X;// 状态值
        Eigen::MatrixXd P;// 状态的协方差矩阵）（相关矩阵），m个特征
        Eigen::MatrixXd U;//
        ////////////
        Eigen::MatrixXd A;// 状态转移矩阵
        Eigen::MatrixXd Q;// 状态转移矩阵的协方差矩阵,模型预测中带来噪声，或是说模型预测后的新的高斯分布，用加法直接体现出来,测量噪声
        Eigen::MatrixXd d_;// 预测距离矩阵
        Eigen::MatrixXd M_dis;// Mahalanobis距离矩阵
        //     static Eigen::Matrix<double, m, 6> H;
        Eigen::MatrixXd r;// 二维距离矩阵
        Eigen::MatrixXd H;// 观测矩阵，传感器参数转换
        Eigen::MatrixXd R;// 观测的噪声方差
        ///////////
        Eigen::MatrixXd X_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd K;
        ///////////
        Eigen::MatrixXd x_tmp;
        Eigen::MatrixXd p_tmp;
        /////////////////

        /***************ekf参数-end***********************/

        /***************ekf-init参数-end*****************/
protected:
};


#endif // KF_H
