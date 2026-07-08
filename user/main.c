#include "stm32f10x.h" // Device header
#include "Delay.h" 
#include "MPU6050.h" 
#include "OLED.h" 
#include "PWM.h" 
#include "Encoder.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motor.h" 
#include "PID.h" 
#include "mpu6050_exti.h" 
#include "NRF24L01.h" 
#include "timer.h" 
#include "NRFRX.h"

extern int flag;
extern uint16_t a,thr,thr_roll,thr_pitch,thr_yaw;//油门;
float Kp=385,Ki=0,Kd=1310; 
float VKp=315,VKi=1.60; 
//float Kp=380,Ki=0,Kd=-900; 
//float VKp=0,VKi=0;
int motor_flag;               //电机使能标志：1使能  0失能
extern float pitch,roll,yaw;
extern float velocity_sum;    //速度的累加
extern int pwm1,pwm2; 
extern int velocity;          //速度测量值（编码器脉冲数，非真实速度）
extern int left_right;
//float Kp=480,Ki=0,Kd=1140; 
//float VKp=320,VKi=0.650;

int main(void)
{
	/*模块初始化*/
	  OLED_Init();		//OLED初始化
    Encoder_IC1();
	  Encoder_IC2();//编码器2
	 MPU6050_Init();
	 MPU6050_DMP_Init();
	 mpu6050_exti();
	 motor_gpio_init();
	
	OLED_ShowString(1,1,"pwm:");
	OLED_ShowString(2,1,"Pitch:");
	OLED_ShowString(3,1,"Roll:");
	OLED_ShowString(4,1,"Yaw:");
//  timer();
	 TIM1_PWM_Init(7200,1-1);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	  GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//????????????PWM????????
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz; 
		GPIO_Init(GPIOC,&GPIO_InitStructure);
		NRF24L01_Init();

	while (1)
	{
//		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//		delay_ms(1000);
//    GPIO_SetBits(GPIOC,GPIO_Pin_13);
//		delay_ms(1000);
     fg();
		if(roll<-30 || roll>30)   //小车可能已经倒了
		{
			motor_flag = 0;         //关闭电机
			velocity_sum = 0;       //速度积分清0
			TIM_SetCounter(TIM3,0); //编码器计数值清零
			TIM_SetCounter(TIM2,0); //编码器计数值清零
		}
		else 
		{
			motor_flag = 1;      //开启电机
		OLED_ShowSignedNum(1,7,left_right,5);
	  OLED_ShowSignedNum(2,7,pwm1, 5);					//OLED显示数据俯角
		OLED_ShowSignedNum(3,7,pwm2, 5);
		OLED_ShowSignedNum(4,7,flag, 5);
		}

	}
}
