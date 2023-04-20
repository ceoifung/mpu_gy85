#include "Gy85.h"

Gy85::Gy85(/* args */)
{
}

Gy85::~Gy85()
{
    adxlBus.closeConnection();
    itgBus.closeConnection();
}

/**
 * @brief 加速度计
 * @param bus 总线地址
 */
int Gy85::initAccel(int bus)
{
    adxlBus = ceoifung::I2cPort(bus, ACCEL_ADDR);
    adxlBus.setBusAddress(bus);
    adxlBus.setDeviceAddress(ACCEL_ADDR);
    int ret = adxlBus.openConnection();
    if (ret != 0)
        return -1;
    adxlBus.writeByte(ACCL_PWR, 0);
    adxlBus.writeByte(ACCL_PWR, 8);
    adxlBus.writeByte(ACCL_DATA_FORMAT, 0);
    adxlBus.writeByte(ACCL_DATA_FORMAT, 11);
    return 0;
}

/**
 * @brief 初始化陀螺仪
 * @param bus 总线地址
 */
int Gy85::initItg3205(int bus)
{
    itgBus = ceoifung::I2cPort(bus, ITG_3205_ADDR);
    itgBus.setBusAddress(bus);
    itgBus.setDeviceAddress(ITG_3205_ADDR);
    int ret = itgBus.openConnection();
    if (ret != 0)
    {
        return -1;
    }
    itgBus.writeByte(PWR_MGMT_1, 0);
    // 0x15
    itgBus.writeByte(ITG_3205_SMPLRT_DIV, 0x07 | 0x00);
    itgBus.writeByte(ITG_3205_DLPF_FS, ITG_3205_FS_SEL_2000_DEG_SEC | 0x01);
    itgBus.writeByte(ITG_3205_INT_CFG, 0x00);
    itgBus.writeByte(0x17, 0x20 | 0x04 | 0x01);
    return 0;
}
/**
 * @brief 读取数据
 * @param type bus类型
 * @param data 读取的数据
 */
int Gy85::readData(int type, short *data)
{
    unsigned char read_buf[6];
    int ret = 0;
    if (type == 1)
    {
        ret = adxlBus.readByteBuffer(ACCEL_XOUT_H, read_buf, 6);
        data[0] = (short)((read_buf[1] << 8) | read_buf[0]);
        data[1] = (short)((read_buf[3] << 8) | read_buf[2]);
        data[2] = (short)((read_buf[5] << 8) | read_buf[4]);
    }
    else if (type == 2)
    {
        // 陀螺仪数据获取
        ret = itgBus.readByteBuffer(ITG_3205_REG_XL, read_buf, 6);
        data[0] = (short)((read_buf[0] << 8) | read_buf[1]);
        data[1] = (short)((read_buf[2] << 8) | read_buf[3]);
        data[2] = (short)((read_buf[4] << 8) | read_buf[5]);
        if (data[0] & (1 << 16 - 1))
            data[0] = data[0] - (1 << 16);
        if (data[1] & (1 << 16 - 1))
            data[1] = data[1] - (1 << 16);
        if (data[2] & (1 << 16 - 1))
            data[2] = data[2] - (1 << 16);
    }

    return ret;
}

/**
 * @brief 读取加速度计数据
 * @param data 读取的数据
 */
int Gy85::readAccelData(short *data)
{
    return readData(1, data);
}

/**
 * @brief 读取陀螺仪数据
 * @param data 读取的数据
 */
int Gy85::readItgData(short *data)
{
    return readData(2, data);
}