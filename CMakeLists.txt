cmake_minimum_required(VERSION 2.8.3)
project(mpu_gy85)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(I2cPort src/I2cPort.cpp)
add_library(Gy85 src/Gy85.cpp)
target_link_libraries( Gy85 ${catkin_LIBRARIES} I2cPort )
add_executable(mpu_gy85_node src/mpu_gy85.cpp)
target_link_libraries(mpu_gy85_node ${catkin_LIBRARIES} I2cPort Gy85)
# target_link_libraries (MPU9255_node wiringPi)

