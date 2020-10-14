cmake_minimum_required(VERSION 3.10)
project(RemoteIIC)

set(CMAKE_CXX_STANDARD 17)

add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h inv_mpu.h inv_mpu_def.h smbus.cpp)