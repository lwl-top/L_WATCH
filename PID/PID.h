#ifndef __PID_H
#define __PID_H


void xianfu(int max);
int vertical_PID_value(float measure,float calcu);
int velocity_PID_value(int velocity,int velocity_caclu);//ËÙ¶È»·£º
int Turn(int gyro_Z,int RC);


#endif
