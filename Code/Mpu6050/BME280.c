#include "stm32f4xx.h"                  // Device header
#include "MPU6050_I2C.h"                  // Device header
#include "BME280.h"                  // Device header

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */

	
void BME280_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MPU6050_Start();						//I2C起始
	MPU6050_SendByte(BME280_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MPU6050_ReceiveAck();					//接收应答
	MPU6050_SendByte(RegAddress);			//发送寄存器地址
	MPU6050_ReceiveAck();					//接收应答
	MPU6050_SendByte(Data);				//发送要写入寄存器的数据
	MPU6050_ReceiveAck();					//接收应答
	MPU6050_Stop();						//I2C终止
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t BME280_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MPU6050_Start();						//I2C起始
	MPU6050_SendByte(BME280_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MPU6050_ReceiveAck();					//接收应答
	MPU6050_SendByte(RegAddress);			//发送寄存器地址
	MPU6050_ReceiveAck();					//接收应答
	
	MPU6050_Start();						//I2C重复起始
	MPU6050_SendByte(BME280_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MPU6050_ReceiveAck();					//接收应答
	Data = MPU6050_ReceiveByte();			//接收指定寄存器的数据
	MPU6050_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MPU6050_Stop();						//I2C终止
	
	return Data;
}


// 读取多字节寄存器
void BME280_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    MPU6050_Start();
    MPU6050_SendByte(BME280_ADDRESS);     // 发送写地址
    MPU6050_ReceiveAck();
    MPU6050_SendByte(reg);                 // 寄存器地址
    MPU6050_ReceiveAck();

    MPU6050_Start();                       // 重复起始
    MPU6050_SendByte(BME280_ADDRESS | 0x01);  // 读地址
    MPU6050_ReceiveAck();

    for(uint8_t i = 0; i < len; i++)
    {
        buf[i] = MPU6050_ReceiveByte();
        if(i == len - 1)
            MPU6050_SendAck(1);            // 最后一个字节NACK
        else
            MPU6050_SendAck(0);            // 其他字节ACK
    }
    MPU6050_Stop();
}
void BME280_sleep(void)
{
	BME280_WriteReg(0xF4, 0x24);
}
void BME280_wake(void)
{
	BME280_WriteReg(0xF4, 0x27);
}
// 温度校准参数
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
int32_t t_fine;  // 计算中间变量

// 读取温度校准参数（0x88开始，6字节）
void BME280_ReadTemperatureCalib(void)
{
uint8_t calib[6];
BME280_ReadRegs(0x88, calib, 6);
dig_T1 = (uint16_t)(calib[0] | (calib[1] << 8));
dig_T2 = (int16_t)(calib[2] | (calib[3] << 8));
dig_T3 = (int16_t)(calib[4] | (calib[5] << 8));
}

// 读取温度原始数据 (0xFA~0xFC)
int32_t BME280_ReadRawTemperature(void)
{
    uint8_t data[3];
    BME280_ReadRegs(0xFA, data, 3);
    int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)(data[2] >> 4));
    return adc_T;
}

// 温度补偿，返回单位是0.01摄氏度
int32_t BME280_CompensateTemperature(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// ----------- 湿度校准参数 -------------
uint8_t  dig_H1;
int16_t dig_H2;
uint8_t  dig_H3;
int16_t dig_H4, dig_H5;
int8_t  dig_H6;

// ----------- 气压校准读取 -------------
void BME280_ReadPressureCalib(void)
{
    uint8_t calib[18];
    BME280_ReadRegs(0x8E, calib, 18);

    dig_P1 = (uint16_t)(calib[0] | (calib[1] << 8));
    dig_P2 = (int16_t)(calib[2] | (calib[3] << 8));
    dig_P3 = (int16_t)(calib[4] | (calib[5] << 8));
    dig_P4 = (int16_t)(calib[6] | (calib[7] << 8));
    dig_P5 = (int16_t)(calib[8] | (calib[9] << 8));
    dig_P6 = (int16_t)(calib[10] | (calib[11] << 8));
    dig_P7 = (int16_t)(calib[12] | (calib[13] << 8));
    dig_P8 = (int16_t)(calib[14] | (calib[15] << 8));
    dig_P9 = (int16_t)(calib[16] | (calib[17] << 8));
}

// ----------- 湿度校准读取 -------------
void BME280_ReadHumidityCalib(void)
{
    uint8_t calib1, calib2[7];
    calib1 = BME280_ReadReg(0xA1);
    BME280_ReadRegs(0xE1, calib2, 7);

    dig_H1 = calib1;
    dig_H2 = (int16_t)(calib2[0] | (calib2[1] << 8));
    dig_H3 = calib2[2];
    dig_H4 = (int16_t)((calib2[3] << 4) | (calib2[4] & 0x0F));
    dig_H5 = (int16_t)((calib2[5] << 4) | (calib2[4] >> 4));
    dig_H6 = (int8_t)calib2[6];
}

// ----------- 原始气压读取 -------------
int32_t BME280_ReadRawPressure(void)
{
    uint8_t data[3];
    BME280_ReadRegs(0xF7, data, 3);
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)(data[2] >> 4));
    return adc_P;
}

// ----------- 原始湿度读取 -------------
int32_t BME280_ReadRawHumidity(void)
{
    uint8_t data[2];
    BME280_ReadRegs(0xFD, data, 2);
    int32_t adc_H = ((int32_t)data[0] << 8) | data[1];
    return adc_H;
}

// ----------- 气压补偿（单位：Pa）-------------
uint32_t BME280_CompensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0; // 避免除以0

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)(p >> 8); // 返回单位：Pa
}

// ----------- 湿度补偿（单位：%RH × 1024）-------------
uint32_t BME280_CompensateHumidity(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) -
        ((int32_t)dig_H5 * v_x1_u32r)) + 16384) >> 15) *
        (((((((v_x1_u32r * (int32_t)dig_H6) >> 10) *
        (((v_x1_u32r * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
        (int32_t)dig_H2 + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12); // 返回单位为：%RH × 1024
}

	
void BME280_Init(void)
{
    BME280_WriteReg(0xF4, 0x27);  
    BME280_WriteReg(0xF5, 0xA0); 
		BME280_ReadTemperatureCalib();
		BME280_ReadPressureCalib();
		BME280_ReadHumidityCalib();	
}