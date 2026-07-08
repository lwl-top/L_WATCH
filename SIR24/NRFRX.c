#include "stm32f10x.h"                  // Device header
#include "NRF24L01.h"

uint32_t JS[6]={0};
uint8_t Buf[32] = {0};	
uint16_t a,thr,thr_roll,thr_pitch,thr_yaw=0;//смце;
int flag;
int left_right;
void nrfx1(void)
{
	NRF24L01_GetRxBuf(Buf);
			JS[1]=Buf[1];
	   thr=JS[1]*6000/255;
}
void fg(void)
{
	if(thr_yaw>5000)
	{
		flag=1;
	}
	else if(thr_yaw<500)
	{
	 flag=2;
	}
	
	if(thr_pitch>5000)
	{
		left_right=1;
	}
	else if(thr_pitch<1000)
	{
		left_right=-1;
	}
	else
		left_right=0;
}
void nrfrx(void)
{
//	  if (NRF24L01_Get_Value_Flag() == 0)
//		{	
		NRF24L01_GetRxBuf(Buf);
			JS[1]=Buf[1];
   		JS[2]=Buf[2];
			JS[3]=Buf[3];
			JS[4]=Buf[4];
   thr=JS[1]*6000/255;
	 thr_yaw=JS[2]*6000/255;
   thr_roll=JS[3]*6000/255;
	 thr_pitch=JS[4]*6000/255;
//	switch (a) {
//    case 1:
//thr = 1000;
////PWM_SetCompare1(100);       
//        break; 
//    case 2:
//thr = 1500;
//    
//        break; 
//		case 3:
//thr = 2000;
//       
//        break; 
//		case 4:
//thr = 2500;
//      
//        break; 
//		case 5:
//thr = 3000;
//      
//        break; 
//		case 6:
//thr = 3500;
//      
//        break; 
//		case 7:
//thr = 4000;
//       
//        break; 
//		case 8:
//thr = 4500;
//       
//        break; 
//		case 9:
//thr = 5000;
//      
//        break; 
//		case 10:
//thr = 5500;
//      
//        break; 
//		case 11:
//thr = 6000;
//		
//        break; 

//    default:
//thr = 0;
// }

//		}
}