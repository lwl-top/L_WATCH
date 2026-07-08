#include "stm32f10x.h"                  // Device header
#include "Delay.h" 

void TIM1_PWM_Init(u16 arr,u16 psc)//TIM1高级定时器初始化，有通道一和通道四口
{  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);	//使能定时器1时钟

	  GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//使用复用推挽输出是为了使PWM波形通过引脚输出
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz; 
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	
  	TIM_InternalClockConfig(TIM1);//选择内部时钟

		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  //初始化TIM1,设置ARR和RCC:
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//初始化TIM1的PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);           //初始化TIM1_CH1  
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);           //初始化TIM1_CH4  
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM1_CH1上的预装载寄存器  
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM1_CH4上的预装载寄存器

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//TIM1是高级定时器，需要总输出使能
	
	TIM_SetCompare1(TIM1,0); //先置零占空比
  TIM_SetCompare4(TIM1,0);
}








