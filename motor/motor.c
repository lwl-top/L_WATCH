#include "stm32f10x.h"                  // Device header
#include "Encoder.h"                 
#include "mpu6050_exti.h"                
#include "PWM.h"                

void PWM_Xianfu(int max ,int *PWM)//PWM的最大输出限制
{
	if(*PWM>max)  * PWM = max;
	if(*PWM<-max) * PWM =-max;
}
void motor_gpio_init(void)//电机驱动引脚初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		
	//正反转控制口初始化：
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //右正反转
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; //左正反转
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void left1(void)//左轮正转
{
	 GPIO_SetBits(GPIOB, GPIO_Pin_6);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_7);

}
void left2(void)//左轮反转
{
	 GPIO_SetBits(GPIOB, GPIO_Pin_7);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_6);

}
void left3(void)//右轮正转
{
	 GPIO_SetBits(GPIOB, GPIO_Pin_13);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_12);

}
void left4(void)//右轮反转
{
	 GPIO_SetBits(GPIOB, GPIO_Pin_12);
	 GPIO_ResetBits(GPIOB, GPIO_Pin_13);

}

void SETPWM(int PWM1,int PWM2)//控制PWM与电机
{
	if(PWM1>0&&PWM2>0) //正转
	{  	
		left1();
		left3();
		TIM_SetCompare1(TIM1,+PWM1);
		TIM_SetCompare4(TIM1,+PWM2);     
	}
	else if(PWM1<0&&PWM2<0)  //反转
	{			
		left2();
    left4();
		TIM_SetCompare1(TIM1,-PWM1); 
		TIM_SetCompare4(TIM1,-PWM2);      
	}	
	else if(PWM1<0&&PWM2>0)
	{
		left2();
		left3();
		TIM_SetCompare1(TIM1,-PWM1); 
		TIM_SetCompare4(TIM1,PWM2);  
	}
	else if(PWM1>0&&PWM2<0)
	{
		left1();
		left4();
		TIM_SetCompare1(TIM1,PWM1); 
		TIM_SetCompare4(TIM1,-PWM2);  
	}
}
int read_encoder3(void)//编码器测速
{
	int speed;
	speed = (short)TIM_GetCounter(TIM3); //1.采集编码器的计数值并保存
	TIM_SetCounter(TIM3,0);              //2.将定时器的计数值清零
	
	return speed;
}

int read_encoder2(void)//编码器测速
{
	int speed;
	speed = (short)TIM_GetCounter(TIM2); //1.采集编码器的计数值并保存
	TIM_SetCounter(TIM2,0);              //2.将定时器的计数值清零
	
	return speed;
}
