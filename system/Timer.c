#include "stm32f10x.h"  // Device header
#include "MPU6050.h" // Device header
uint16_t numberr;
void timer(void)
	{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//开启定时时钟
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 ;
		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz; 

		GPIO_Init(GPIOB,&GPIO_InitStructure);
//	TIM_InternalClockConfig(TIM2);//选择内部时钟
		TIM_ETRClockMode2Config(TIM4,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted ,0x00);//采用ETR时钟外部通道2,外部输入触发计数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//用来配置时机单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1  ;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up  ;//计数器模式，向上计数
	TIM_TimeBaseInitStructure.TIM_Period=1800-1;//ARR自动重装器的值
	TIM_TimeBaseInitStructure.TIM_Prescaler=720-1;//预分频器的取值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复重装器，高级定时器才用到
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
		
		
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn  ;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(& NVIC_InitStructure);	
	TIM_Cmd(TIM4,ENABLE);
	
}
	uint16_t GETCOUNTER(void)
	{
		return TIM_GetCounter(TIM4);
	}
	uint16_t numbe(void)
{
	return numberr;
}
void TIM2_IRQHandler (void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
