cmake_minimum_required(VERSION 3.10)
project(RemoteIIC)

set(CMAKE_CXX_STANDARD 17)
#add_definitions(-DINV_IMU_DEBUG=1)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_Release "${CMAKE_C_FLAGS_Release} -O3")

add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h smbus.cpp smbus.h
        drv_imu_invensense.hpp drv_imu_invensense.cpp drv_imu_invensense_def.hpp drv_imu_invensense_port.hpp)