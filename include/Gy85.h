#ifndef __GY85__H__
#define __GY85__H__

#include "I2cPort.h"
#include <iostream>

class Gy85
{
private:
    ceoifung::I2cPort *adxlBus;
    ceoifung::I2cPort *itgBus;
    // 加速度计的i2c从机地址
    uint8_t ACCEL_ADDR = 0x53; // i2c设备地址
    // 电源管理
    uint8_t ACCL_PWR = 0x2d;
    // data format输出格式
    uint8_t ACCL_DATA_FORMAT = 0x31;
    uint8_t ACCEL_XOUT_H = 0x32; // AXH, 0x32
    uint8_t ACCEL_XOUT_L = 0x33; // AXL, 0x33
    uint8_t ACCEL_YOUT_H = 0x34; // AYH, 0x34
    uint8_t ACCEL_YOUT_L = 0x35; // AYL
    uint8_t ACCEL_ZOUT_H = 0x36; // AZH
    uint8_t ACCEL_ZOUT_L = 0x37; // AZL
    // 陀螺仪寄存器配置
    int ITG_3205_SMPLRT_DIV = 0x15; // 0x15
    int ITG_3205_DLPF_FS = 0x16; // 0x16
    int ITG_3205_FS_SEL_2000_DEG_SEC = 0x18; // 0x18
    int ITG_3205_DLPF_CFG_188_1 = 0x01; // 0x01
    int ITG_3205_INT_CFG = 0x17; // 0x17
    int ITG_3205_ADDR = 0x68; // i2c地址
    // 温度
    int ITG_3205_TEMP_OUT_H = 0x1B;
    int ITG_3205_TEMP_OUT_L = 0x1C;
    // 陀螺仪数据
    int ITG_3205_REG_XL = 0x1d; // 陀螺仪x轴低字节寄存器
    int ITG_3205_REG_XH = 0x1e; // 陀螺仪x轴高字节寄存器
    int ITG_3205_REG_YL = 0x1f; // 陀螺仪y轴低字节寄存器
    int ITG_3205_REG_YH = 0x20; // 陀螺仪y轴高字节寄存器
    int ITG_3205_REG_ZL = 0x21; // 陀螺仪z轴低字节寄存器
    int ITG_3205_REG_ZH = 0x22; // 陀螺仪z轴高字节寄存器
    int PWR_MGMT_1 = 0x3E;      // 电源管理
    int readData(int, short*);
public:
    Gy85(/* args */);
    ~Gy85();
    int initAccel(int bus);
    int initItg3205(int bus);
    int readAccelData(short *);
    int readItgData(short *);
};

#endif