cmake_minimum_required(VERSION 3.10)

SET(CMAKE_C_COMPILER /usr/bin/clang)
SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

SET (CMAKE_C_FLAGS                "-Wall -std=c99")
SET (CMAKE_C_FLAGS_DEBUG          "-O0 -g")
SET (CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g")

SET (CMAKE_CXX_FLAGS                "-Wall -std=c++17")
SET (CMAKE_CXX_FLAGS_DEBUG          "-O0 -g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

#add_definitions(-DINV_IMU_DEBUG=1)

project(RemoteIIC)



add_executable(RemoteIIC main.cpp remote_i2c.cpp remote_i2c.h smbus.cpp smbus.h
        drv_imu_invensense.hpp drv_imu_invensense.cpp drv_imu_invensense_def.hpp drv_imu_invensense_port.hpp
        ahrs/ahrs.cpp ahrs/ahrsEigen.cpp
        ahrs/ahrs.h ahrs/ahrsEigen.h
        )