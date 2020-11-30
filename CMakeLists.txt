cmake_minimum_required(VERSION 3.10)

#以下指令无效
#在命令行中执行export CC=/usr/bin/clang
#在命令行中执行export CXX=/usr/bin/clang++
#SET(CMAKE_C_COMPILER /usr/bin/clang)
#SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

#SET (CMAKE_C_FLAGS                "-W -std=c99")
SET (CMAKE_C_FLAGS                "-Wall -std=c99")
SET (CMAKE_C_FLAGS_DEBUG          "-O0 -g")
SET (CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g")

#SET (CMAKE_CXX_FLAGS                "-W -std=c++17")
SET (CMAKE_CXX_FLAGS                "-Wall -Wreorder -std=c++17")
SET (CMAKE_CXX_FLAGS_DEBUG          "-O0 -g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

#add_definitions(-DINV_IMU_DEBUG=1)
add_definitions(-DINV_YES_TRACE)

project(RemoteIIC)



add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h smbus.cpp smbus.h
        inv_imu.hpp inv_imu.cpp
        )