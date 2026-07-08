#ifndef __PWM_H
#define __PWM_H

void PWM(void);
void compare(uint16_t counter);
void speed(int8_t count);
void TIM1_PWM_Init(u16 arr,u16 psc);


#endif
