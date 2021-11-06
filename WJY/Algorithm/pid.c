#include "pid.h"
#include "Peripherals_can.h" 
#include "math.h"
PID_Typedef incremental_pid_angle,incremental_pid_speed,incremental_pid_current,position_pid_angle,position_pid_speed,angle_pid;
int32_t target_angle[8],target_speed[8],target_current[8] = {0};
int32_t Encoder_values,Last_Encoder_values = 0;
int16_t circle_cnt = 0;
float k_gain,k_restrain,High_error_margin,Low_error_margin,allowed_minimum_error = 0;


float Incremental_PID(PID_Typedef  *ptr,int32_t measured,int32_t target,float kp,float ki,float kd)
{ 
//传入数据
  ptr->kp = kp;
  ptr->ki = ki;
  ptr->kd = kd;	
  ptr->Target  = target;
  ptr->Measure = measured;
  ptr->error = ptr->Target - ptr->Measure;
	
  ptr->kp_out = kp*(ptr->error - ptr->last_error);
  ptr->ki_out = ki*ptr->error;
  ptr->kd_out = kd*(ptr->error - 2*ptr->last_error + ptr->last_last_error);
		

  ptr->output_value += ptr->kp_out + ptr->ki_out + ptr->kd_out;
  
  ptr->last_error = ptr->error;
  ptr->last_last_error = ptr->last_error;
	
	
  if(ptr->output_value >= 16384)
  {
    ptr->output_value = 16384;
  }
  else if(ptr->output_value < -16384)
  {
  ptr->output_value = -16384;
  }
  return  ptr->output_value;
}

float Position_PID(PID_Typedef  *ptr,int32_t measured,int32_t target,float kp,float ki,float kd){
//传入数据
  ptr->kp = kp;
  ptr->ki = ki;
  ptr->kd = kd;	
  ptr->Target  = target;
  ptr->Measure = measured;
  ptr->error = ptr->Target - ptr->Measure;
  
//开始进行pid算法    
  ptr->kp_out = ptr->kp*ptr->error;	
  //积分分离
  //if((unsigned)(ptr->error) < 500){
  ptr->ki_out += ptr->ki*ptr->error;
  //}
  ptr->kd_out = ptr->kd*(ptr->error - ptr->last_error);
  ptr->output_value =  ptr->kp_out + ptr->ki_out + ptr->kd_out;
  
  ptr->last_error = ptr->error;  
  
 //限制输出，不能超过输出范围
  if(ptr->output_value >= 16384){ptr->output_value = 16384;}
  else if(ptr->output_value < -16384){ptr->output_value = -16384;}
  
  return  ptr->output_value;
}

float  Expert_PID(PID_Typedef *ptr,float Measure,float Target,float KP, float KI, float KD, float k_gain, float k_restrain, float High_error_margin,float Low_error_margin, float allowed_minimum_error) {

	float uk1,uk2 = 0;
	ptr->kp = KP;
	ptr->ki = KI;
	ptr->kd = KD;

	ptr->error = Target - Measure;
	if(ptr->error > ptr->High_error_margin) {
		uk1 = ptr->kp * ptr->error;
	} else {
		if(ptr->error * (ptr->error - ptr->last_error) > 0) {
			if((unsigned)(ptr->error) >= ptr->Low_error_margin ) {
				uk1 = uk2 + ptr->k_gain *(ptr->kp * (ptr->error - ptr->last_error) +
				                          ptr->ki * ptr->error +
				                          ptr->kd * (ptr->error - 2*ptr->last_error +ptr->last_last_error ));
			} else {
				uk1 = uk2 + ptr->kp * (ptr->error - ptr->last_error) +
				      ptr->ki * ptr->error +
				      ptr->kd * (ptr->error - 2 * ptr->last_error +ptr->last_last_error );
			} 
		} else if(ptr->error * (ptr->error - ptr->last_error) < 0) {
			if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) > 0) {
			} else if((ptr->error - ptr->last_error) * (ptr->last_error - ptr->last_last_error ) < 0) { 
				if((unsigned)(ptr->error) >= ptr->Low_error_margin) {
					uk1 = uk2 + ptr->k_gain * ptr->kp * ptr->last_error ;
				} else {
					uk1 = uk2 + ptr->k_restrain * ptr->kp * ptr->last_error ;
				}

			}

		}
		else if( fabs(ptr->error) < ptr->allowed_minimum_error){
			uk1 += (0.5f * ptr->error);
		}

	}

	uk2 = uk1;
	ptr->last_error = ptr->error;

	ptr->last_last_error = ptr->last_error;
	  if(uk1 >= 30000){uk1 = 30000;}
  else if(uk1 < -30000){uk1 = -30000;}
	
	return uk1;

}









