#include <stdio.h>
#include <stdint.h>
#include "Gy85.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

int main(int argc, char **argv)
{
      ros::init(argc, argv, "IMU_pub");
      ros::NodeHandle n;
      ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
      int bus = -1;
      ros::param::get("~bus", bus);
      if (bus == -1)
            bus = 1;
      ROS_INFO("init imu...\n");
      Gy85 sensor;
      if (sensor.initAccel(bus) != 0)
      {
            ROS_INFO("Accelerometer init error...\n");
            exit(0);
            return 0;
      }
      if (sensor.initItg3205(bus) != 0)
      {
            ROS_INFO("Gyroscope init error...\n");
            exit(0);
            return 0;
      }

      short accBuffer[3] = {0};
      short itgBuffer[3] = {0};
      while (ros::ok())
      {
            sensor_msgs::Imu data_imu;
            data_imu.header.stamp = ros::Time::now();
            data_imu.header.frame_id = "imu_link";

            float conversion_gyro = 3.1415 / (180.0 * 32.8f);
            float conversion_acce = 9.8 / 16384.0f;

            if (sensor.readAccelData(accBuffer) == 0)
            {
                  //  std::cout << "读取到加速度计数据: " << std::endl;
                  // std::cout << "Ax:" << buf[0] << "\t";
                  // std::cout << "Ay:" << buf[1] << "\t";
                  // std::cout << "Az:" << buf[2] << std::endl;
                  data_imu.linear_acceleration.x = accBuffer[0] * conversion_acce;
                  data_imu.linear_acceleration.y = accBuffer[1] * conversion_acce;
                  data_imu.linear_acceleration.z = accBuffer[2] * conversion_acce;
            }
            if (sensor.readItgData(itgBuffer) == 0)
            {
                  // 打印读取到的数据，单位是mg（毫重力）
                  // std::cout << "读取到加速度计数据: " << std::endl;
                  // std::cout << "Gx:" << InBuffer[0] << "\t";
                  // std::cout << "Gy:" << InBuffer[1] << "\t";
                  // std::cout << "Gz:" << InBuffer[2] << "\t";
                  data_imu.angular_velocity.x = itgBuffer[0] * conversion_gyro;
                  data_imu.angular_velocity.y = itgBuffer[1] * conversion_gyro;
                  data_imu.angular_velocity.z = itgBuffer[2] * conversion_gyro;
            }
            pub_imu.publish(data_imu);
            ros::spinOnce();
      }
      return 0;
}
