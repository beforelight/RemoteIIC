cmake_minimum_required(VERSION 3.10)
project(RemoteIIC)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DINV_IMU_DEBUG)
add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h drv_imu_invensense.hpp drv_imu_invensense.cpp drv_imu_invensense_def.hpp smbus.cpp drv_imu_invensense.cpp)