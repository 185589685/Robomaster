#include "pid.h"
#include "Peripherals_can.h" 
#include "math.h"
#include "typedef.h"
//PID_Typedef incremental_pid_angle,incremental_pid_speed,incremental_pid_current,position_pid_angle,position_pid_speed,angle_pid;
//int32_t target_angle[8],target_speed[8],target_current[8] = {0};
int32_t Encoder_values,Last_Encoder_values = 0;
int16_t circle_cnt = 0;
Motor_t M3508[8] = {0};
//float k_gain,k_restrain,High_error_margin,Low_error_margin,allowed_minimum_error = 0;

float Zero_Treated(float ture_value){
   Encoder_values = ture_value;
   
 //过零处理：码盘值 - 上一次码盘值 > 4096 判断已经过0
  if(Encoder_values - Last_Encoder_values > +6000){ 
	  circle_cnt--;
  }
  if(Encoder_values - Last_Encoder_values < -6000 ){
      circle_cnt++;
  }
  Last_Encoder_values = Encoder_values;
  if(circle_cnt == 32767){
   circle_cnt = 0;
  }
  return circle_cnt;
}

void Motor_para_Init(){
for(int i = 0;i < 8;i++){
	M3508[i].realAngle = 0;
	M3508[i].realSpeed = 0;
	M3508[i].realCurrent = 0;
	M3508[i].temperture = 0;
	M3508[i].position_pid.kp = 0;
	M3508[i].position_pid.ki = 0;
	M3508[i].position_pid.kd = 0;
	M3508[i].position_pid.kp_out = 0;
	M3508[i].position_pid.ki_out = 0;
	M3508[i].position_pid.kd_out = 0;
	M3508[i].position_pid.Measure = 0;
	M3508[i].position_pid.Target = 0;
	M3508[i].position_pid.error = 0;	
	M3508[i].position_pid.last_error= 0;
	M3508[i].position_pid.last_last_error = 0;	
	M3508[i].position_pid.output_value = 0;
	
 }
//return M3508;
}
float Incremental_PID(Motor_t *ptr,int32_t measured,int32_t target,float kp,float ki,float kd)
{ 
//传入数据
  ptr->incremental_pid.kp = kp;
  ptr->incremental_pid.ki = ki;
  ptr->incremental_pid.kd = kd;	
  ptr->incremental_pid.Target  = target;
  ptr->incremental_pid.Measure = measured;
  ptr->incremental_pid.error = ptr->incremental_pid.Target - ptr->incremental_pid.Measure;
	
  ptr->incremental_pid.kp_out = kp*(ptr->incremental_pid.error - ptr->incremental_pid.last_error);
  ptr->incremental_pid.ki_out = ki*ptr->incremental_pid.error;
  ptr->incremental_pid.kd_out = kd*(ptr->incremental_pid.error - 2*ptr->incremental_pid.last_error + ptr->incremental_pid.last_last_error);
		

  ptr->incremental_pid.output_value += ptr->incremental_pid.kp_out + ptr->incremental_pid.ki_out + ptr->incremental_pid.kd_out;
  
  ptr->incremental_pid.last_error = ptr->incremental_pid.error;
  ptr->incremental_pid.last_last_error = ptr->incremental_pid.last_error;
	
	
  if(ptr->incremental_pid.output_value >= 6600)
  {
    ptr->incremental_pid.output_value = 6600;
  }
  else if(ptr->incremental_pid.output_value < -6600)
  {
  ptr->incremental_pid.output_value = -6600;
  }
  return  ptr->incremental_pid.output_value;
}

float Position_PID(Motor_t *ptr,int32_t measured,int32_t target,float kp,float ki,float kd){
//传入数据
  ptr->position_pid.kp = kp;
  ptr->position_pid.ki = ki;
  ptr->position_pid.kd = kd;	
  ptr->position_pid.Target  = target;
  ptr->position_pid.Measure = measured;
  ptr->position_pid.error = ptr->position_pid.Target - ptr->position_pid.Measure;
  
//开始进行pid算法    
  ptr->position_pid.kp_out = ptr->position_pid.kp*ptr->position_pid.error;	
  //积分分离
  //if((unsigned)(ptr->error) < 500){
  ptr->position_pid.ki_out += ptr->position_pid.ki*ptr->position_pid.error;
  //}
  ptr->position_pid.kd_out = ptr->position_pid.kd*(ptr->position_pid.error - ptr->position_pid.last_error);
  ptr->position_pid.output_value =  ptr->position_pid.kp_out + ptr->position_pid.ki_out + ptr->position_pid.kd_out;
  
  ptr->position_pid.last_error = ptr->position_pid.error;  
  
 //限制输出，不能超过输出范围
  if(ptr->position_pid.output_value >= 6600){ptr->position_pid.output_value = 6600;}
  else if(ptr->position_pid.output_value < -6600){ptr->position_pid.output_value = -6600;}
  
  return  ptr->position_pid.output_value;
}

//float  Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, float High_error_margin,float Low_error_margin, float allowed_minimum_error) {

//	float uk1,uk2 = 0;
//	ptr->kp = KP;
//	ptr->ki = KI;
//	ptr->kd = KD;

//	ptr->error = Target - Measure;
//	if(ptr->error > ptr->High_error_margin) {
//		uk1 = ptr->kp * ptr->error;
//	} else {
//		if(ptr->error * (ptr->error - ptr->last_error) > 0) {
//			if((unsigned)(ptr->error) >= ptr->Low_error_margin ) {
//				uk1 = uk2 + ptr->k_gain *(ptr->kp * (ptr->error - ptr->last_error) +
//				                          ptr->ki * ptr->error +
//				                          ptr->kd * (ptr->error - 2*ptr->last_error +ptr->last_last_error ));
//			} else {
//				uk1 = uk2 + ptr->kp * (ptr->error - ptr->last_error) +
//				      ptr->ki * ptr->error +
//				      ptr->kd * (ptr->error - 2 * ptr->last_error +ptr->last_last_error );
//			} 
//		} else if(ptr->error * (ptr->error - ptr->last_error) < 0) {
//			if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) > 0) {
//			} else if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) < 0) { 
//				if((unsigned)(ptr->error) >= ptr->Low_error_margin) {
//					uk1 = uk2 + ptr->k_gain * ptr->kp * ptr->last_error ;
//				} else {
//					uk1 = uk2 + ptr->k_restrain * ptr->kp * ptr->last_error ;
//				}

//			}

//		}
//		else if( fabs(ptr->error) < ptr->allowed_minimum_error){
//			uk1 += (0.5f * ptr->error);
//		}

//	}

//	uk2 = uk1;
//	ptr->last_error = ptr->error;

//	ptr->last_last_error = ptr->last_error;
//	  if(uk1 >= 30000){uk1 = 30000;}
//  else if(uk1 < -30000){uk1 = -30000;}
//	
//	return uk1;

//}









