#include "pid.h"
extern float Kp,Ki,Kd;   //直立环参数
float err;               //此次误差
float last_err;          //上次误差
float err_sum=0;         //误差累加
float err_difference;    //误差的差值

extern float VKp,VKi;    //速度环参数
float filt_velocity;     //滤波后的速度
float last_filt_velocity;//上一次的滤波后的速度
float velocity_sum=0;    //速度的累加
float Turn_Kp=-30;
extern float  Turn_Kd;
  

int vertical_PID_value(float measure,float calcu)//直立环：

{

	err = calcu - measure;             //误差
	err_sum+=err;                      //误差的累加
	err_difference = err - last_err;   //误差的差值
	last_err = err;                    //此次误差记录为“上次误差”
	
	return Kp*err + Ki*err_sum + Kd*err_difference;
}


int velocity_PID_value(int velocity,int velocity_caclu)//速度环：

{
	float a=0.2;                                           //滤波系数（反映滤波程度）
	filt_velocity = a*(velocity-velocity_caclu) + (1-a)*last_filt_velocity; //一阶速度滤波
	velocity_sum +=  filt_velocity;                        //速度的累加
	xianfu(5000);                                        //累加限幅
	last_filt_velocity = filt_velocity;                    //此次速度记录为“上次速度”

	return VKp*filt_velocity + VKi*velocity_sum;
}

int Turn(int gyro_Z,int RC)
{
  int PWM_out; 
  PWM_out = Turn_Kd*gyro_Z+Turn_Kp*RC;
  return PWM_out;
}

void xianfu(int max)//对速度环限幅：

{
	if(velocity_sum>max)  velocity_sum=max;
	if(velocity_sum<-max) velocity_sum=-max;
}
