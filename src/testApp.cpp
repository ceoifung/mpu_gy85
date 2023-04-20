
#include "Gy85.h"

int main()
{
    Gy85 sensor;
    if (sensor.initAccel(1) != 0)
    {
        std::cerr << "初始化加速度计传感器失败" << std::endl;
    }
    if (sensor.initItg3205(1) != 0)
    {
        std::cerr << "初始化陀螺仪传感器失败" << std::endl;
    }

    short buf[3];
    short buf1[3];
    while (true)
    {
        if (sensor.readItgData(buf1) == 0)
        {
            std::cout << "Gx:" << buf1[0] << "\t";
            std::cout << "Gy:" << buf1[1] << "\t";
            std::cout << "Gz:" << buf1[2] << "\t";
        }
        if (sensor.readAccelData(buf) == 0)
        {
            std::cout << "Ax:" << buf[0] << "\t";
            std::cout << "Ay:" << buf[1] << "\t";
            std::cout << "Az:" << buf[2] << "\n";
        }
        
        usleep(1000000); // Wait for 100 milliseconds
    }
    return 0;
}