#ifndef __MOTOR_H
#define __MOTOR_H
int read_encoder2(void);
int read_encoder3(void);
void PWM_Xianfu(int max,int *PWM);
void SETPWM(int PWM1,int PWM2);//控制PWM与电机
void left1(void);//左轮正转
void left2(void);//左轮反转
void left3(void);//右轮正转
void left4(void);//右轮反转
void motor_gpio_init(void);




#endif
