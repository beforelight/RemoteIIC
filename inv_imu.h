//
// Created by 17616 on 2020/10/12.
//

#ifndef REMOTEIIC_INV_IMU_H
#define REMOTEIIC_INV_IMU_H

#include <cstdint>
#include <string>
#include <memory>
#include "inv_mpu_def.h"
#ifdef INV_IMU_DEBUG
#ifdef __linux__
#include<iostream>
#define INV_PRINTF printf
#endif
#else
#define INV_PRINTF(...)
#endif
namespace inv {
    class i2c_interface {
    public:
        i2c_interface(void *_context,
                      int (*_read)(void *context,
                                   unsigned char addr, unsigned char reg,
                                   unsigned char *val, unsigned int len),
                      int (*_write)(void *context,
                                    unsigned char addr, unsigned char reg,
                                    const unsigned char *val, unsigned int len),
                      int (*_readNonBlocking)(void *context,
                                              unsigned char addr, unsigned char reg,
                                              unsigned char *val, unsigned int len),
                      int (*_writeNonBlocking)(void *context,
                                               unsigned char addr, unsigned char reg,
                                               const unsigned char *val, unsigned int len))
                : context(_context), read(_read), write(_write),
                  readNonBlocking(_readNonBlocking), writeNonBlocking(_writeNonBlocking) {}

        int Read(unsigned char addr, unsigned char reg,
                 unsigned char *val, unsigned int len) {
            return (*read)(context, addr, reg, val, len);
        }

        int Write(unsigned char addr, unsigned char reg,
                  const unsigned char *val, unsigned int len) {
            return (*write)(context, addr, reg, val, len);
        }

        int ReadNonBlocking(unsigned char addr, unsigned char reg,
                            unsigned char *val, unsigned int len) {
            return (*readNonBlocking)(context, addr, reg, val, len);
        }

        int WriteNonBlocking(unsigned char addr, unsigned char reg,
                             const unsigned char *val, unsigned int len) {
            return (*writeNonBlocking)(context, addr, reg, val, len);
        }

    protected:
        void *context;
        int (*read)(void *context,
                    unsigned char addr, unsigned char reg,
                    unsigned char *val, unsigned int len);
        int (*write)(void *context,
                     unsigned char addr, unsigned char reg,
                     const unsigned char *val, unsigned int len);
        int (*readNonBlocking)(void *context,
                               unsigned char addr, unsigned char reg,
                               unsigned char *val, unsigned int len);
        int (*writeNonBlocking)(void *context,
                                unsigned char addr, unsigned char reg,
                                const unsigned char *val, unsigned int len);
    };

    struct config {
        enum mpu_accel_fs {    // In the ACCEL_CONFIG (0x1C) register, the full scale select  bits are :
            MPU_FS_2G = 0,    // 00 = 2G
            MPU_FS_4G,        // 01 = 4
            MPU_FS_8G,        // 10 = 8
            MPU_FS_16G,        // 11 = 16
            NUM_MPU_AFS
        } accel_fs;

        /** @brief Allowed value for accel DLPF bandwidth (ACCEL_CONFIG2 (0x1D) register) */
        enum mpu_accel_bw {        // In the ACCEL_CONFIG2 (0x1D) register, the BW setting bits are :
            MPU_ABW_218 = 1,    ///< 001 = 218 Hz
            MPU_ABW_99,            ///< 010 = 99 Hz
            MPU_ABW_45,            ///< 011 = 45 Hz
            MPU_ABW_21,            ///< 100 = 21 Hz
            MPU_ABW_10,            ///< 101 = 10 Hz
            MPU_ABW_5,            ///< 110 = 5 Hz
            MPU_ABW_420,        ///< 111 = 420 Hz
            NUM_MPU_ABW
        } accel_bw;

        enum mpu_gyro_fs {        // In the GYRO_CONFIG register, the fS_SEL bits are :
            MPU_FS_250dps = 0,    // 00 = 250
            MPU_FS_500dps,        // 01 = 500
            MPU_FS_1000dps,        // 10 = 1000
            MPU_FS_2000dps,        // 11 = 2000
            NUM_MPU_GFS
        } gyro_fs;

        /** @brief Allowed value for gyro DLPF bandwidth (CONFIG (0x1A) register) */
        enum mpu_gyro_bw {   // In the CONFIG register, the  BW setting bits are :
            MPU_GBW_250 = 0, ///< 000 = 250 Hz
            MPU_GBW_176 = 1, ///< 001 = 176 Hz
            MPU_GBW_92,         ///< 010 = 92 Hz
            MPU_GBW_41,         ///< 011 = 41 Hz
            MPU_GBW_20,         ///< 100 = 20 Hz
            MPU_GBW_10,         ///< 101 = 10 Hz
            MPU_GBW_5,         ///< 110 = 5 Hz
            NUM_MPU_GBW
        } gyro_bw;

        config(mpu_accel_fs _accel_fs = MPU_FS_8G, mpu_accel_bw _accel_bw = MPU_ABW_99,
               mpu_gyro_fs _gyro_gs = MPU_FS_2000dps, mpu_gyro_bw _gyro_bw = MPU_GBW_92) :
                accel_fs(_accel_fs), accel_bw(_accel_bw),
                gyro_fs(_gyro_gs), gyro_bw(_gyro_bw) {}
    };

    class imu {
    public:
        virtual int init(config _cfg = config()) = 0;
        virtual bool detect() = 0;
        virtual bool self_test() = 0;
        virtual void set_bias() = 0;
        virtual int converter(float *acc_x, float *acc_y, float *acc_z,
                              float *gyro_x, float *gyro_y, float *gyro_z) = 0;
        virtual int converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                              int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) = 0;
        virtual int converter(float *mag_x, float *mag_y, float *mag_z) = 0;
        virtual int converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) = 0;
        virtual int converter(float *temp) = 0;
        virtual int read_sensor_blocking() = 0;
        virtual int read_sensor_NonBlocking() = 0;
        virtual std::string report() = 0;
        virtual int soft_reset(void) = 0;

    public:
        bool is_open() { return isOpen; };

    protected:
        imu(i2c_interface &_i2c) : i2c(_i2c), isOpen(false), addr(0), cfg(config()) {}

        int write_reg(uint8_t reg, const uint8_t val) { return i2c.Write(addr, reg, &val, 1); };

        int read_reg(uint8_t reg, uint8_t *val) { return i2c.Read(addr, reg, val, 1); };

    protected:
        uint8_t addr;
        bool isOpen;
        config cfg;
        i2c_interface &i2c;
    };

    class icm20602 : public imu {
    public:
        icm20602(i2c_interface &_i2c);
        int init(config _cfg = config());
        bool detect();
        bool self_test();
        void set_bias();
        int converter(float *acc_x, float *acc_y, float *acc_z,
                      float *gyro_x, float *gyro_y, float *gyro_z);
        int converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                      int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
        int converter(float *mag_x, float *mag_y, float *mag_z);
        int converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
        int converter(float *temp);
        int read_sensor_blocking();
        int read_sensor_NonBlocking();
        std::string report();
        int soft_reset(void);

    public:
        int data_rdy();

    public:
        const int DEF_ST_PRECISION = 1000;
        const int DEF_GYRO_CT_SHIFT_DELTA = 500;
        const int DEF_ACCEL_ST_SHIFT_DELTA = 500;
        /* Gyro Offset Max Value (dps) */
        const int DEF_GYRO_OFFSET_MAX = 20;
        /* Gyro Self Test Absolute Limits ST_AL (dps) */
        const int DEF_GYRO_ST_AL = 60;
        /* Accel Self Test Absolute Limits ST_AL (mg) */
        const int DEF_ACCEL_ST_AL_MIN = 225;
        const int DEF_ACCEL_ST_AL_MAX = 675;

    protected:
        float accel_unit;
        float gyro_unit;
        uint8_t buf[14];
    };

    class mpu6050 : public icm20602 {
    public:
        mpu6050(i2c_interface &_i2c) : icm20602(_i2c) {}
        int init(config _cfg = config());
        bool detect();
        bool self_test();
        void set_bias();
        int converter(float *acc_x, float *acc_y, float *acc_z,
                      float *gyro_x, float *gyro_y, float *gyro_z);
        int converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                      int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
        int converter(float *mag_x, float *mag_y, float *mag_z);
        int converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
        int converter(float *temp);
        int read_sensor_blocking();
        int read_sensor_NonBlocking();
        std::string report();
        int soft_reset(void);

    public:
    };


    class mpu9250 : public icm20602 {
    public:
        mpu9250(i2c_interface &_i2c);
        int init(config _cfg = config());
        bool detect();
        bool self_test();
        void set_bias();
        int converter(float *acc_x, float *acc_y, float *acc_z,
                      float *gyro_x, float *gyro_y, float *gyro_z);
        int converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                      int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
        int converter(float *mag_x, float *mag_y, float *mag_z);
        int converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
        int converter(float *temp);
        int read_sensor_blocking();
        int read_sensor_NonBlocking();
        std::string report();
        int soft_reset(void);

    public:
        int sub_i2c_read(unsigned char addr,
                         unsigned char reg,
                         unsigned char *val,
                         unsigned int len = 1);
        int sub_i2c_write(unsigned char addr,
                          unsigned char reg,
                          const unsigned char *val,
                          unsigned int len = 1);
    public:
        const int MPU9250_I2C_SLV4_EN = 0x80;
        const int MPU9250_I2C_SLV4_DONE = 0x40;
        const int MPU9250_I2C_SLV4_NACK = 0x10;
        const int MPU9250_AK8963_I2C_ADDR = 0x0C;
        const int MPU9250_AK8963_POWER_DOWN = 0x10;
        const int MPU9250_AK8963_FUSE_ROM_ACCESS = 0x1F;
        const int MPU9250_AK8963_SINGLE_MEASUREMENT = 0x11;
        const int MPU9250_AK8963_CONTINUOUS_MEASUREMENT = 0x16; //MODE 2
        const int MPU9250_AK8963_DATA_READY = (0x01);
        const int MPU9250_AK8963_DATA_OVERRUN = (0x02);
        //#define MPU9250_AK8963_OVERFLOW        (0x80)
        const int MPU9250_AK8963_OVERFLOW = (0x08);
        const int MPU9250_AK8963_DATA_ERROR = (0x40);
        const int MPU9250_AK8963_CNTL2_SRST = 0x01;

    private:
        uint8_t buf[22];
        uint8_t ak8963_DeviceID;
        uint8_t ak8963_Information;
        const float mag_unit = 0.15f;;//固定量程4900uT 0.15µT/LSB
        float AK8963_ASA[3];
    };
    int Parser(i2c_interface &_i2c, std::shared_ptr<imu> &ptr);
}
#endif //REMOTEIIC_INV_IMU_H
