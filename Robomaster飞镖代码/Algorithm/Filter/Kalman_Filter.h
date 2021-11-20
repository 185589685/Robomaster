#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H
//�ṹ�����Ͷ���
typedef struct 
{
    float LastP;//�ϴι���Э����  
    float Now_P;//��ǰ����Э����  
    float out;  //�������˲������ 
    float Kg;   //����������         
    float Q;    //��������Э����  
    float R;    //�۲�����Э����   
}KFP;           //Kalman Filter parameter
extern KFP KFP_height[8];
float KalmanFilter(KFP *kfp,float input);

#endif



