//
// Created by 17616 on 2020/10/12.
//

#ifndef REMOTEIIC_DRV_IMU_INVENSENSE_HPP
#define REMOTEIIC_DRV_IMU_INVENSENSE_HPP

#include <cstdint>
#include <string>
#include <memory>
#include "drv_imu_invensense_def.hpp"
#ifdef INV_IMU_DEBUG
#ifdef __linux__
#include<iostream>
#define INV_PRINTF printf
#endif
#else
#define INV_PRINTF(...)
#endif
namespace inv {
    class i2cInterface_t {
    public:
        i2cInterface_t(void *_context,
                       int (*_readBlocking)(void *context,
                                            unsigned char addr, unsigned char reg,
                                            unsigned char *val, unsigned int len),
                       int (*_writeBlocking)(void *context,
                                             unsigned char addr, unsigned char reg,
                                             const unsigned char *val, unsigned int len),
                       int (*_readNonBlocking)(void *context,
                                               unsigned char addr, unsigned char reg,
                                               unsigned char *val, unsigned int len),
                       int (*_writeNonBlocking)(void *context,
                                                unsigned char addr, unsigned char reg,
                                                const unsigned char *val, unsigned int len))
                : context(_context), readBlocking(_readBlocking), writeBlocking(_writeBlocking),
                  readNonBlocking(_readNonBlocking), writeNonBlocking(_writeNonBlocking) {}

        int ReadBlocking(unsigned char addr, unsigned char reg,
                         unsigned char *val, unsigned int len) {
            return (*readBlocking)(context, addr, reg, val, len);
        }

        int WriteBlocking(unsigned char addr, unsigned char reg,
                          const unsigned char *val, unsigned int len) {
            return (*writeBlocking)(context, addr, reg, val, len);
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
        int (*readBlocking)(void *context,
                            unsigned char addr, unsigned char reg,
                            unsigned char *val, unsigned int len);
        int (*writeBlocking)(void *context,
                             unsigned char addr, unsigned char reg,
                             const unsigned char *val, unsigned int len);
        int (*readNonBlocking)(void *context,
                               unsigned char addr, unsigned char reg,
                               unsigned char *val, unsigned int len);
        int (*writeNonBlocking)(void *context,
                                unsigned char addr, unsigned char reg,
                                const unsigned char *val, unsigned int len);
    };

    struct config_t {
        enum mpu_accel_fs {    // In the ACCEL_CONFIG (0x1C) register, the full scale select  bits are :
            MPU_FS_2G = 0,    // 00 = 2G
            MPU_FS_4G,        // 01 = 4
            MPU_FS_8G,        // 10 = 8
            MPU_FS_16G,        // 11 = 16
            NUM_MPU_AFS
        } accelFullScale;

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
        } accelBandwidth;

        enum mpu_gyro_fs {        // In the GYRO_CONFIG register, the fS_SEL bits are :
            MPU_FS_250dps = 0,    // 00 = 250
            MPU_FS_500dps,        // 01 = 500
            MPU_FS_1000dps,        // 10 = 1000
            MPU_FS_2000dps,        // 11 = 2000
            NUM_MPU_GFS
        } gyroFullScale;

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
        } gyroBandwidth;

        config_t(mpu_accel_fs _accel_fs = MPU_FS_8G, mpu_accel_bw _accel_bw = MPU_ABW_99,
                 mpu_gyro_fs _gyro_gs = MPU_FS_2000dps, mpu_gyro_bw _gyro_bw = MPU_GBW_92) :
                accelFullScale(_accel_fs), accelBandwidth(_accel_bw),
                gyroFullScale(_gyro_gs), gyroBandwidth(_gyro_bw) {}
    };

    class imu_t {
    public:
        virtual int Init(config_t _cfg = config_t()) = 0;
        virtual bool Detect() = 0;
        virtual int SelfTest() = 0;
        virtual int Converter(float *acc_x, float *acc_y, float *acc_z,
                              float *gyro_x, float *gyro_y, float *gyro_z) = 0;
        virtual int Converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                              int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) = 0;
        virtual int Converter(float *mag_x, float *mag_y, float *mag_z) = 0;
        virtual int Converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) = 0;
        virtual int Converter(float *temp) = 0;
        virtual int ReadSensorBlocking() = 0;
        virtual int ReadSensorNonBlocking() = 0;
        virtual std::string Report() = 0;

    public:
        bool IsOpen() { return isOpen; };

        void SetConfig(config_t _cfg) { cfg = _cfg; }

        constexpr const config_t &GetConfig() { return cfg; }

        void SetI2cAddr(uint8_t _addr) { addr = _addr; }

        constexpr const uint8_t &GetI2cAddr() { return addr; }

    protected:
        void SetIsOpen() { isOpen = true; }

        void ClearIsOpen() { isOpen = false; }

        imu_t(i2cInterface_t &_i2c) : i2c(_i2c), isOpen(false), addr(0), cfg(config_t()) {}

        i2cInterface_t &i2c;

        int WriteReg(uint8_t reg, const uint8_t val) { return i2c.WriteBlocking(addr, reg, &val, 1); };

        int ReadReg(uint8_t reg, uint8_t *val) { return i2c.ReadBlocking(addr, reg, val, 1); };

        int ModifyReg(uint8_t reg, const uint8_t val, const uint8_t mask) {
            uint8_t regVal;
            int res = 0;
            res |= ReadReg(reg, &regVal);
            res |= WriteReg(reg, (regVal & (~mask)) | (val & mask));
            return res;
        }

    private:
        uint8_t addr;
        bool isOpen;
        config_t cfg;
    };

    class mpuxxxx_t : public imu_t {
    public:
        int Init(config_t _cfg = config_t()) override;

        bool Detect() override { return false; }

        int SelfTest() override { return 0; }

        int Converter(float *acc_x, float *acc_y, float *acc_z,
                      float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                      int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Converter(float *mag_x, float *mag_y, float *mag_z);
        int Converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;

        int Converter(float *temp) override { return 0; }

        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;

        std::string Report() override { return std::string(); }

    public:
        virtual int SoftReset(void) = 0;
        virtual int EnableDataReadyInt();
        virtual bool DataReady();


    protected:
        mpuxxxx_t(i2cInterface_t &_i2c);
        float accelUnit;
        float gyroUnit;
        uint8_t buf[14];
    };


    class mpu6050_t : public mpuxxxx_t {
    public:
        mpu6050_t(i2cInterface_t &_i2c) : mpuxxxx_t(_i2c) {}

        bool Detect() override;
        int SelfTest() override;
        int Converter(float *temp) override;
        std::string Report() override;
        int SoftReset(void) override;
    };

    class mpu6500Series_t : public mpuxxxx_t {
    protected:
        mpu6500Series_t(i2cInterface_t &_i2c) : mpuxxxx_t(_i2c) {}

    public:
        int SelfTest() override;
        virtual uint8_t REG_SELF_TEST_X_ACCEL() = 0;
        virtual uint8_t REG_SELF_TEST_X_GYRO() = 0;
        constexpr static int DEF_ST_PRECISION = 1000;
        constexpr static int DEF_GYRO_CT_SHIFT_DELTA = 500;
        const int DEF_ACCEL_ST_SHIFT_DELTA = 500;
        /* Gyro Offset Max Value (dps) */
        constexpr static int DEF_GYRO_OFFSET_MAX = 20;
        /* Gyro Self Test Absolute Limits ST_AL (dps) */
        constexpr static int DEF_GYRO_ST_AL = 60;
        /* Accel Self Test Absolute Limits ST_AL (mg) */
        constexpr static int DEF_ACCEL_ST_AL_MIN = 225;
        constexpr static int DEF_ACCEL_ST_AL_MAX = 675;
    };

    class icm20602_t : public mpu6500Series_t {
    public:
        icm20602_t(i2cInterface_t &_i2c) : mpu6500Series_t(_i2c) {}

        int SoftReset(void) override;
        bool Detect() override;
        int Converter(float *temp) override;
        std::string Report() override;

        uint8_t REG_SELF_TEST_X_ACCEL() override { return (uint8_t) icm20602_RegMap::SELF_TEST_X_ACCEL; }

        uint8_t REG_SELF_TEST_X_GYRO() override { return (uint8_t) icm20602_RegMap::SELF_TEST_X_GYRO; }
    };

    class mpu9250_t : public mpu6500Series_t {
    public:
        mpu9250_t(i2cInterface_t &_i2c);
        int Init(config_t _cfg = config_t()) override;
        bool Detect() override;
        int Converter(float *acc_x, float *acc_y, float *acc_z,
                      float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                      int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Converter(float *mag_x, float *mag_y, float *mag_z) override;
        int Converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
        int Converter(float *temp) override;
        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;
        std::string Report() override;
        int SoftReset(void) override;

        uint8_t REG_SELF_TEST_X_ACCEL() override { return (uint8_t) mpu9250_RegMap::SELF_TEST_X_ACCEL; }

        uint8_t REG_SELF_TEST_X_GYRO() override { return (uint8_t) mpu9250_RegMap::SELF_TEST_X_GYRO; }

    public:
        int SubI2cRead(unsigned char addr,
                       unsigned char reg,
                       unsigned char *val,
                       unsigned int len = 1);
        int SubI2cWrite(unsigned char addr,
                        unsigned char reg,
                        const unsigned char *val,
                        unsigned int len = 1);
    public:
        constexpr static int MPU9250_I2C_SLV4_EN = 0x80;
        constexpr static int MPU9250_I2C_SLV4_DONE = 0x40;
        constexpr static int MPU9250_I2C_SLV4_NACK = 0x10;
        constexpr static int MPU9250_AK8963_I2C_ADDR = 0x0C;
        constexpr static int MPU9250_AK8963_POWER_DOWN = 0x10;
        constexpr static int MPU9250_AK8963_FUSE_ROM_ACCESS = 0x1F;
        constexpr static int MPU9250_AK8963_SINGLE_MEASUREMENT = 0x11;
        constexpr static int MPU9250_AK8963_CONTINUOUS_MEASUREMENT = 0x16; //MODE 2
        constexpr static int MPU9250_AK8963_DATA_READY = (0x01);
        constexpr static int MPU9250_AK8963_DATA_OVERRUN = (0x02);
        //constexpr static int MPU9250_AK8963_OVERFLOW = (0x80);
        constexpr static int MPU9250_AK8963_OVERFLOW = (0x08);
        constexpr static int MPU9250_AK8963_DATA_ERROR = (0x40);
        constexpr static int MPU9250_AK8963_CNTL2_SRST = 0x01;

    private:
        uint8_t buf[22];
        uint8_t ak8963DeviceId;
        uint8_t ak8963Information;
        const float magUnit = 0.15f;;//固定量程4900uT 0.15µT/LSB
        float ak8963Asa[3];
    };

    class imuPtr_t : public std::shared_ptr<imu_t> {
    public:
        int Load(i2cInterface_t &_i2c);
    };
}
#endif //REMOTEIIC_DRV_IMU_INVENSENSE_HPP
