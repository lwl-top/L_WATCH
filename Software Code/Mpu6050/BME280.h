#ifndef __BME280_I2C_H
#define __BME280_I2C_H


#define BME280_ADDRESS              	0x76<<1

/*REGISTERS*/
#define	BME280_REGISTER_DIG_T1      	0x88
#define	BME280_REGISTER_DIG_T2      	0x8A
#define	BME280_REGISTER_DIG_T3      	0x8C

#define	BME280_REGISTER_DIG_P1      	0x8E
#define	BME280_REGISTER_DIG_P2			0x90
#define	BME280_REGISTER_DIG_P3      	0x92
#define	BME280_REGISTER_DIG_P4      	0x94
#define	BME280_REGISTER_DIG_P5      	0x96
#define	BME280_REGISTER_DIG_P6          0x98
#define	BME280_REGISTER_DIG_P7          0x9A
#define	BME280_REGISTER_DIG_P8          0x9C
#define	BME280_REGISTER_DIG_P9          0x9E

#define	BME280_REGISTER_DIG_H1          0xA1
#define	BME280_REGISTER_DIG_H2          0xE1
#define	BME280_REGISTER_DIG_H3          0xE3
#define	BME280_REGISTER_DIG_H4          0xE4
#define	BME280_REGISTER_DIG_H5          0xE5
#define	BME280_REGISTER_DIG_H6          0xE7

#define	BME280_REGISTER_CHIPID          0xD0
#define	BME280_REGISTER_VERSION         0xD1
#define	BME280_REGISTER_SOFTRESET       0xE0

#define	BME280_REGISTER_CAL26           0xE1  // R calibration stored in 0xE1-0xF0

#define	BME280_REGISTER_CONTROLHUMID	  0xF2
#define	BME280_REGISTER_STATUS          0XF3
#define	BME280_REGISTER_CONTROL         0xF4
#define	BME280_REGISTER_CONFIG          0xF5
#define	BME280_REGISTER_PRESSUREDATA    0xF7
#define	BME280_REGISTER_TEMPDATA        0xFA
#define	BME280_REGISTER_HUMIDDATA       0xFD

void BME280_sleep(void);
void BME280_wake(void);
void BME280_ReadTemperatureCalib(void);
int32_t BME280_ReadRawTemperature(void);
int32_t BME280_CompensateTemperature(int32_t adc_T);
int32_t BME280_ReadRawPressure(void);
int32_t BME280_ReadRawHumidity(void);
uint32_t BME280_CompensatePressure(int32_t adc_P);
uint32_t BME280_CompensateHumidity(int32_t adc_H);
void BME280_Init(void);

#endif
