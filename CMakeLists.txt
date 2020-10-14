cmake_minimum_required(VERSION 3.10)
project(RemoteIIC)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DINV_IMU_DEBUG)
add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h inv_imu.h inv_imu.cpp inv_mpu_def.h smbus.cpp inv_imu.cpp)