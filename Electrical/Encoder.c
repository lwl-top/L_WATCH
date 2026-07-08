#include "stm32f10x.h"                  // Device header
#include "Delay.h" 

void Encoder_IC1(void)//编码器1
{
   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);//开启定时时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	  GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//用来配置时机单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1  ;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up  ;//计数器模式，向上计数
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//ARR自动重装器的值
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;//预分频器的取值PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复重装器，高级定时器才用到
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);//给结构体赋一个初始值
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1 ;//用于配置TIM2的通道一
	TIM_ICInitStructure.TIM_ICFilter=0xF;//用于配置滤波
   TIM_ICInit(TIM2,	&TIM_ICInitStructure);
	 
	 TIM_ICInitStructure.TIM_Channel=TIM_Channel_2 ;//用于配置TIM2的通道二
	 TIM_ICInitStructure.TIM_ICFilter=0xF;//用于配置滤波
	 TIM_ICInit(TIM2,	&TIM_ICInitStructure);
	 TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
			TIM_SetCounter(TIM2,0);//清零定时器计数值

		TIM_Cmd(TIM2,ENABLE);	
}

//uint16_t getcnt(void)//获取CNT的值用来读取一定的速度,可以设定一个时间段，在时间段内计数CNT的值，然后把CNT清零，
//{
//	uint16_t temp;
//	temp=TIM_GetCounter(TIM2);
//	TIM_SetCounter(TIM2,0);
////	Delay_ms(1000);	
//	return temp;	//返回CNT的值	

//}
void Encoder_IC2(void)//编码器2
{
   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);//开启定时时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//用来配置时机单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1  ;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up  ;//计数器模式，向上计数
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//ARR自动重装器的值
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;//预分频器的取值PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复重装器，高级定时器才用到
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);//给结构体赋一个初始值
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1 ;//用于配置TIM3的通道一
	TIM_ICInitStructure.TIM_ICFilter=0xF;//用于配置滤波
   TIM_ICInit(TIM3,	&TIM_ICInitStructure);
	 
	 TIM_ICInitStructure.TIM_Channel=TIM_Channel_2 ;//用于配置TIM3的通道二
	 TIM_ICInitStructure.TIM_ICFilter=0xF;//用于配置滤波
	 TIM_ICInit(TIM3,	&TIM_ICInitStructure);
	 TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	 			TIM_SetCounter(TIM3,0);//清零定时器计数值

	 TIM_Cmd(TIM3,ENABLE);	
}
