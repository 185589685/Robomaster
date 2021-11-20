#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H
//结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差  
    float Now_P;//当前估算协方差  
    float out;  //卡尔曼滤波器输出 
    float Kg;   //卡尔曼增益         
    float Q;    //过程噪声协方差  
    float R;    //观测噪声协方差   
}KFP;           //Kalman Filter parameter
extern KFP KFP_height[8];
float KalmanFilter(KFP *kfp,float input);

#endif



