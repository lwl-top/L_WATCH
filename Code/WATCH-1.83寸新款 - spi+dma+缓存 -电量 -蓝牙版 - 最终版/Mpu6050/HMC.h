#ifndef _HMC_H_
#define _HMC_H_
#include "MPU6050_I2C.h"

#define HMC_ADDRESS 0X3C    
void HMC_sleep(void);
void HMC_wake(void);
void HMC_Init(void);
void HMC_READ_DATA(int16_t *data);

#endif
