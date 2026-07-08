#include "stm32f10x.h"  // Device header
#include "MPU6050.h" 
#include "OLED.h"
#include "NRF24L01.h"
#include "pid.h"
#include "NRFRX.h"

uint8_t flag_2ms=0;
uint8_t flag_4ms=0;
uint8_t flag_8ms=0;
uint8_t flag_30ms=0;
uint8_t flag_60ms=0;

void timer(void)
	{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//开启定时时钟
//	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//		GPIO_InitTypeDef GPIO_InitStructure;
//		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
//		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 ;
//		GPIO_InitStructure.GPIO_Speed=  GPIO_Speed_50MHz; 

//		GPIO_Init(GPIOA,&GPIO_InitStructure);
	TIM_InternalClockConfig(TIM4);//选择内部时钟
//		TIM_ETRClockMode2Config(TIM2,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted ,0x00);//采用ETR时钟外部通道2,外部输入触发计数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//用来配置时机单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1  ;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up  ;//计数器模式，向上计数
	TIM_TimeBaseInitStructure.TIM_Period=500-1;//ARR自动重装器的值
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//预分频器的取值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复重装器，高级定时器才用到
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
		
		
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn  ;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);	
	TIM_Cmd(TIM4,ENABLE);
	
}


void TIM4_IRQHandler (void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
	{
		static uint8_t state2ms=0;
		static uint8_t state4ms=0;
		static uint8_t state8ms=0;
		static uint16_t state30ms=0;
		static uint16_t state60ms=0;

     state2ms++;
		 state4ms++;
		 state8ms++;
		 state30ms++;
		 state60ms++;

		if(state2ms==1)//1ms更新一次
		{
			state2ms=0;		
      flag_2ms=1;			
//	   Prepare_Data(); 
//	   IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle);  
//			nrfrx();
//		  Angle_Cal();		 
//			Control(&Att_Angle,&Gyr_rad,1);

		}
		if(state4ms==2)//1ms更新一次
		{		 
			state4ms=0;
		 	flag_4ms=1;

		}
		if(state8ms==4)//1ms更新一次
		{		 
			state8ms=0;
		 	flag_8ms=1;

//      nrfrx();	
		}
			if(state30ms==8)//1ms更新一次
		{		 
			state30ms=0;
		 	flag_30ms=1;

//      nrfrx();	
		}
			if(state60ms==2000)//1ms更新一次
		{		 
			state60ms=0;
		 	flag_60ms=1;

//      nrfrx();	
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
