#include "stm32f10x.h"                  // Device header
#include "Delay.h" 
#include "MPU6050.h" 
#include "PWM.h"
#include "Encoder.h" 
#include "inv_mpu.h"
#include "motor.h" 
#include "PID.h" 
#include "NRF24L01.h" 
#include "NRFRX.h" 

extern uint16_t a,thr,thr_roll,thr_pitch,thr_yaw;//油门;
float pitch,roll,yaw;
float measure,calcu;   //测量值和理论值                   
int velocity;          //速度测量值（编码器脉冲数，非真实速度）
int pwm1,pwm2;               //PWM值
extern int motor_flag;
extern int flag;
short gx,gy,gz;
float turn_speed=0;
extern int left_right;
float Turn_Kd;

void turn_speed_xianfu(int speed)
{
	if(speed>100)turn_speed=100;
	if(speed<-100)turn_speed=-100;
}
void mpu6050_exti(void)//mpu6050中断函数
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	

	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
		
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE ;
	EXTI_InitStructure.EXTI_Mode=  EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Falling;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel= EXTI15_10_IRQn   ;	
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
		
  NVIC_Init	(&NVIC_InitStructure);
}
void EXTI15_10_IRQHandler(void)
{

		 MPU6050_DMP_Get_Data(&pitch,&roll,&yaw);//获取各个轴的角度
				MPU_Get_Gyroscope(&gx,&gy,&gz);
			measure = roll;                                     //roll测量值
		   calcu=0;	//roll理论值	
			velocity = ( read_encoder2()-read_encoder3() )/2; 
			 
				if(flag==1)
	{
	 pwm1=pwm2= vertical_PID_value(measure, calcu) + velocity_PID_value(velocity,0);    //PID计算：直立环+速度环
	}
      else if(flag==2)
			 {
				 if(left_right==1){turn_speed++,Turn_Kd=0,turn_speed_xianfu(turn_speed);}
			 else if(left_right==-1){turn_speed--,Turn_Kd=0,turn_speed_xianfu(turn_speed);}
       else    turn_speed=0,Turn_Kd=-0.8;
			 pwm1 =pwm2= vertical_PID_value(measure, calcu) + velocity_PID_value(velocity,(thr_roll-2940)/120);    //PID计算：直立环+速度环
			 pwm1=pwm1-Turn(gz,turn_speed);
			 pwm2=pwm2+Turn(gz,turn_speed);
			 }
		  PWM_Xianfu(7000,&pwm1);      //PWM限幅
			PWM_Xianfu(7000,&pwm2);      //PWM限幅
      if(motor_flag) SETPWM(pwm1,pwm2); //给电机PWM
			else           SETPWM(0,0);   //关闭电机	
      nrfrx();	
	 EXTI_ClearFlag(EXTI_Line12);
}	

