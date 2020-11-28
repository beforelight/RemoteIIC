/**
 * @brief 陀螺仪驱动，适用于mpu6050,mpu9250,icm20602
 * @author xiao qq1761690868
 * @doc drv_imu_invensense.md
 * @version v1.0
 * @date 2020-10-16
 */

#ifndef UTILITIES_DRV_IMU_INVENSENSE_HPP
#define UTILITIES_DRV_IMU_INVENSENSE_HPP

#include <cstdint>
#include <cstdlib>
#include <string>
#include <memory>
#include <functional>
#include "drv_imu_invensense_port.hpp"

#ifdef INV_PRINTF

#if (defined(HITSIC_INV_IMU_DEBUG) && (HITSIC_INV_IMU_DEBUG > 0))

#if (defined(HITSIC_INV_YES_TRACE) && (HITSIC_INV_YES_TRACE > 0))
#define INV_TRACE_(fmt, ...) \
    INV_PRINTF("[I]imu_inv line:%d:trace: " fmt "%s\r\n",  __LINE__, __VA_ARGS__)
#define INV_TRACE(...) INV_TRACE_(__VA_ARGS__, "")
#else
#define INV_TRACE(...)
#endif//(defined(HITSIC_INV_YES_TRACE)&&(HITSIC_INV_YES_TRACE>0))

#if !(defined(HITSIC_INV_NO_DEBUG) && (HITSIC_INV_NO_DEBUG > 0))
#define INV_DEBUG_(fmt, ...) \
    INV_PRINTF("[E]imu_inv line:%d:debug: " fmt "%s\r\n",  __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) INV_DEBUG_(__VA_ARGS__, "")
#else
#define INV_DEBUG(...)
#endif//!(defined(HITSIC_INV_NO_DEBUG)&&(HITSIC_INV_NO_DEBUG>0))

#endif//(defined(HITSIC_INV_IMU_DEBUG)&&(HITSIC_INV_IMU_DEBUG>0))

#endif //INV_PRINTF

#if defined(__linux__)
#include<iostream>
#undef INV_TRACE_
#undef INV_TRACE
#undef INV_DEBUG_
#undef INV_DEBUG
#undef HITSIC_INV_IMU_DEBUG
#define HITSIC_INV_IMU_DEBUG 1
#define INV_TRACE_(fmt, ...) \
    printf("%s:%d:trace: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_TRACE(...) INV_TRACE_(__VA_ARGS__, "")
#define INV_DEBUG_(fmt, ...) \
    printf("%s:%d:debug: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) INV_DEBUG_(__VA_ARGS__, "")
#endif//defined(__linux__)


namespace inv {


    //i2c接口
    class i2cInterface_t {
    public:
        /**
         * @param  _context          :调用函数指针传入的用户参数
         * @param  _readBlocking     :约定如下，阻塞读
         *  **************************************************
         *  * @brief   这里的函数指针的参数约定
         *  * @param  {void*}                : 用户参数
         *  * @param  {uint8_t}        : iic从机地址
         *  * @param  {uint8_t}        : 从机寄存器地址
         *  * @param  {const unsigned* char} : 缓存地址
         *  * @param  {unsigned int}         : 数据长度
         *  * @return {int}                  : 错误码
         *  **************************************************
         * @param  _writeBlocking    :约定同上，阻塞写
         * @param  _readNonBlocking  :约定同上，非阻塞读
         */
        i2cInterface_t(void *_context,
                       int (*_readBlocking)(void *context,
                                            uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len),
                       int (*_writeBlocking)(void *context,
                                             uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len),
                       int (*_readNonBlocking)(void *context,
                                               uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len))
                : context(_context), readBlocking(_readBlocking), writeBlocking(_writeBlocking),
                  readNonBlocking(_readNonBlocking) {}

        i2cInterface_t(void *_context,
                       int (*_readBlocking)(void *context,
                                            uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len),
                       int (*_writeBlocking)(void *context,
                                             uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len)) :
                i2cInterface_t(_context, _readBlocking, _writeBlocking, _readBlocking) {}


        void *context;
        int (*readBlocking)(void *context,
                            uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);
        int (*writeBlocking)(void *context,
                             uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len);
        int (*readNonBlocking)(void *context,
                               uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);
    };

#if (__cplusplus > 201103L)
    class i2cif_t : public i2cInterface_t {
    private:
        std::function<int(uint8_t, uint8_t, uint8_t *, unsigned int)> read;
        std::function<int(uint8_t, uint8_t, uint8_t *, unsigned int)> readNB;
        std::function<int(uint8_t, uint8_t, const uint8_t *, unsigned int)> write;
        static int ReadBlocking(void *context,
                                uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len) {
            return static_cast<i2cif_t *>(context)->read(addr, reg, val, len);
        }
        static int WriteBlocking(void *context,
                                 uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len) {
            return static_cast<i2cif_t *>(context)->write(addr, reg, val, len);
        }
        static int ReadNonBlocking(void *context,
                                   uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len) {
            return static_cast<i2cif_t *>(context)->readNB(addr, reg, val, len);
        }
    public:
        i2cif_t(std::function<int(uint8_t, uint8_t, uint8_t *, unsigned int)> _read,
                std::function<int(uint8_t, uint8_t, const uint8_t *, unsigned int)> _write,
                std::function<int(uint8_t, uint8_t, uint8_t *, unsigned int)> _readNonBlocking) :
                read(_read), readNB(_readNonBlocking), write(_write),
                i2cInterface_t(this, ReadBlocking, WriteBlocking, ReadNonBlocking) {}
        i2cif_t(std::function<int(uint8_t, uint8_t, uint8_t *, unsigned int)> _read,
                std::function<int(uint8_t, uint8_t, const uint8_t *, unsigned int)> _write)
                : i2cif_t(_read, _write, _read) {}
    };
    class spiif_t : public i2cInterface_t {
    private:
//        void *realloc(void *mem_address, unsigned int newsize);
        uint8_t *txbuff;
        uint8_t *rxbuff;
        int *shareCounter;
        int buffsize;
        std::function<int(const uint8_t *, uint8_t *, unsigned int)> transfer;
        static int ReadBlocking(void *context,
                                uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len) {
            spiif_t *spi = static_cast<spiif_t *>(context);
            if (len + 1 > spi->buffsize) {
                if ((*spi->shareCounter) == 0) {
                    free(spi->txbuff);
                    free(spi->rxbuff);
                } else {
                    (*spi->shareCounter)--;
                }
                spi->txbuff = (uint8_t *) malloc(len + 30);
                spi->rxbuff = (uint8_t *) malloc(len + 30);
                spi->buffsize = len + 30;
            }
            spi->txbuff[0] = (1 << 8) | (reg & 0x7f);
            int rtv = spi->transfer(spi->txbuff, spi->rxbuff, len + 1);
            memcpy(val, &spi->rxbuff[1], len);
            return rtv;
        }
        static int WriteBlocking(void *context,
                                 uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len) {
            spiif_t *spi = static_cast<spiif_t *>(context);
            if (len + 1 > spi->buffsize) {
                if ((*spi->shareCounter) == 0) {
                    free(spi->txbuff);
                    free(spi->rxbuff);
                } else {
                    (*spi->shareCounter)--;
                }
                spi->txbuff = (uint8_t *) malloc(len + 30);
                spi->rxbuff = (uint8_t *) malloc(len + 30);
                spi->buffsize = len + 30;
            }
            spi->txbuff[0] = (reg & 0x7f);
            memcpy(&spi->txbuff[1], val, len);
            int rtv = spi->transfer(spi->txbuff, spi->rxbuff, len + 1);
            return rtv;
        }
    public:
        spiif_t(std::function<int(const uint8_t *, uint8_t *, unsigned int)> transfer_)
                : transfer(transfer_), shareCounter(new int(0)), i2cInterface_t(this, ReadBlocking, WriteBlocking) {
            txbuff = (uint8_t *) malloc(30);
            rxbuff = (uint8_t *) malloc(30);
            buffsize = 30;
        }
        spiif_t(const spiif_t &r) : i2cInterface_t(this, ReadBlocking, WriteBlocking) {
            rxbuff = r.rxbuff;
            txbuff = r.txbuff;
            shareCounter = r.shareCounter;
            buffsize = r.buffsize;
            transfer = r.transfer;
            (*shareCounter)++;
        };
        ~spiif_t() {
            if (*shareCounter == 0) {
                free(txbuff);
                free(rxbuff);
                delete shareCounter;
            } else {
                (*shareCounter)--;
            }
        }
    };
#endif
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

        enum mpu_gyro_unit {
            MPU_UNIT_DegPerSec = 0,
            MPU_UNIT_RadPerSec = 1,
            MPU_UNIT_RevolutionsPerMinute,
        } gyroUnit;

        enum mpu_accel_unit {
            MPU_UNIT_MetersPerSquareSecond = 0,
            MPU_UNIT_G,
            MPU_UNIT_mG
        } accelUnit;

        config_t(mpu_accel_fs _accel_fs = MPU_FS_8G, mpu_accel_bw _accel_bw = MPU_ABW_99,
                 mpu_accel_unit mpuAccelUnit = MPU_UNIT_MetersPerSquareSecond,
                 mpu_gyro_fs _gyro_gs = MPU_FS_2000dps, mpu_gyro_bw _gyro_bw = MPU_GBW_92,
                 mpu_gyro_unit mpuGyroUnit = MPU_UNIT_DegPerSec)
                : accelFullScale(_accel_fs), accelBandwidth(_accel_bw), accelUnit(mpuAccelUnit),
                  gyroFullScale(_gyro_gs), gyroBandwidth(_gyro_bw), gyroUnit(mpuGyroUnit) {}
    };


    class imu_t {
    public:
        virtual ~imu_t() {}
        virtual int Init(config_t _cfg = config_t()) = 0;
        virtual bool Detect() = 0;
        virtual int SelfTest() = 0;
        virtual std::string Report() = 0;
        virtual bool DataReady() = 0;
        virtual int EnableDataReadyInt() = 0;
        virtual int SoftReset() = 0;
        virtual int ReadSensorBlocking() = 0;
        virtual int ReadSensorNonBlocking() = 0;
        virtual int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) = 0;
        virtual int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) = 0;
        virtual int Convert(float *mag_x, float *mag_y, float *mag_z) = 0;
        virtual int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) = 0;
        virtual int Convert(float *temp) = 0;
    public:
        static constexpr const uint8_t AddrAutoDetect = 0;
        imu_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : i2c(_i2c), isOpen(false), addr(_addr), cfg(config_t()) {
            if (addr == AddrAutoDetect) {
                addrAutoDetect = 0;
            } else {
                addrAutoDetect = 1;
            }
        }
        bool IsOpen() { return isOpen; };
        void SetConfig(config_t _cfg) { cfg = _cfg; }
        constexpr const config_t &GetConfig() { return cfg; }
        void SetI2cAddr(uint8_t _addr) { if (addrAutoDetect == 0) { addr = _addr; }}
        constexpr const uint8_t &GetI2cAddr() { return addr; }
        int WriteReg(uint8_t reg, const uint8_t val);
        int WriteRegVerified(uint8_t reg, const uint8_t val);
        int ReadReg(uint8_t reg, uint8_t *val);
        int ModifyReg(uint8_t reg, const uint8_t val, const uint8_t mask);
    protected:
        void SetIsOpen() { isOpen = true; }
        void ClearIsOpen() { isOpen = false; }
        i2cInterface_t &i2c;
    private:
        int addrAutoDetect;
        uint8_t addr;
        bool isOpen;
        config_t cfg;
    };

    class mpu6050_t : public imu_t {
    public:
        ~mpu6050_t() {}
        mpu6050_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : imu_t(_i2c, _addr) {}

        int Init(config_t _cfg = config_t()) override;
        bool Detect() override;
        int SelfTest() override;
        std::string Report() override;
        bool DataReady() override;
        int EnableDataReadyInt() override;
        int SoftReset() override;
        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;
        int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Convert(float *mag_x, float *mag_y, float *mag_z) override;
        int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
        int Convert(float *temp) override;

    private:
        float gyroUnit;
        float accelUnit;
        uint8_t buf[14];
    };

    class icm20602_t : public imu_t {
    public:
        virtual ~icm20602_t() {}
        icm20602_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : imu_t(_i2c, _addr) {}

        int Init(config_t _cfg = config_t()) override;
        bool Detect() override;
        int SelfTest() override;
        std::string Report() override;
        bool DataReady() override;
        int EnableDataReadyInt() override;
        int SoftReset() override;
        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;
        int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Convert(float *mag_x, float *mag_y, float *mag_z) override;
        int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
        int Convert(float *temp) override;
    private:
        float gyroUnit;
        float accelUnit;
        uint8_t buf[14];
    };

    class icm20600_t : public icm20602_t {
    public:
    public:
        virtual ~icm20600_t() {}
        icm20600_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : icm20602_t(_i2c, _addr) {}
        bool Detect() override;
        std::string Report() override;
    };

    class mpu9250_t : public imu_t {
    public:
        virtual ~mpu9250_t() {}

        mpu9250_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : imu_t(_i2c, _addr) {}

        int Init(config_t _cfg = config_t()) override;
        bool Detect() override;
        int SelfTest() override;
        std::string Report() override;
        bool DataReady() override;
        int EnableDataReadyInt() override;
        int SoftReset() override;
        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;
        int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Convert(float *mag_x, float *mag_y, float *mag_z) override;
        int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
        int Convert(float *temp) override;
    public:
        int SubI2cRead(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len = 1);
        int SubI2cWrite(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len = 1);
    private:
        uint8_t buf[22];
        float gyroUnit;
        float accelUnit;
        uint8_t ak8963DeviceId;
        uint8_t ak8963Information;
        constexpr static float magUnit = 0.15f;;//固定量程4900uT 0.15µT/LSB
        float ak8963Asa[3];
    };

    class icm20948_t : public imu_t {
    public:
        ~icm20948_t() {}
        icm20948_t(i2cInterface_t &_i2c, uint8_t _addr = AddrAutoDetect) : imu_t(_i2c, _addr), bank(0) {}

        int Init(config_t _cfg = config_t()) override;
        bool Detect() override;
        int SelfTest() override;
        std::string Report() override;
        bool DataReady() override;
        int EnableDataReadyInt() override;
        int SoftReset() override;
        int ReadSensorBlocking() override;
        int ReadSensorNonBlocking() override;
        int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) override;
        int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
        int Convert(float *mag_x, float *mag_y, float *mag_z) override;
        int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
        int Convert(float *temp) override;
    public:
        int SubI2cRead(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len = 1);
        int SubI2cWrite(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len = 1);
        int WriteReg(uint16_t reg, const uint8_t val);
        int WriteRegVerified(uint16_t reg, const uint8_t val);
        int ReadReg(uint16_t reg, uint8_t *val);
        int ModifyReg(uint16_t reg, const uint8_t val, const uint8_t mask);
    private:
        int SwitchBank(int _bank);
    private:
        int bank;
        float gyroUnit;
        float accelUnit;
        uint8_t ak09916DeviceId;
        uint8_t buf[14 + 9];
        constexpr static float magUnit = 0.15f;;//固定量程4900uT 0.15µT/LSB
    };


    class imuPtr_t : public std::shared_ptr<imu_t> {
    public:
        int Load(i2cInterface_t &_i2c, uint8_t _addr = imu_t::AddrAutoDetect);
    };

    /*
     * 以下是常量定义
     * 以下是常量定义
     * 以下是常量定义
     * 以下是常量定义
     * 以下是常量定义
     * 以下是常量定义
     * 以下是常量定义
     */
#if 1
    enum class icm20602_RegMap :uint8_t
    {
        XG_OFFS_TC_H = 0x4,			// READ/ WRITE
        XG_OFFS_TC_L = 0x5,			// READ/ WRITE
        YG_OFFS_TC_H = 0x7,			// READ/ WRITE
        YG_OFFS_TC_L = 0x8,			// READ/ WRITE
        ZG_OFFS_TC_H = 0x0A,		// READ/ WRITE
        ZG_OFFS_TC_L = 0x0B,		// READ/ WRITE
        SELF_TEST_X_ACCEL = 0x0D,	// READ/ WRITE
        SELF_TEST_Y_ACCEL = 0x0E,	// READ/ WRITE
        SELF_TEST_Z_ACCEL = 0x0F,	// READ/ WRITE
        XG_OFFS_USRH = 0x13,		// READ/ WRITE
        XG_OFFS_USRL = 0x14,		// READ/ WRITE
        YG_OFFS_USRH = 0x15,		// READ/ WRITE
        YG_OFFS_USRL = 0x16,		// READ/ WRITE
        ZG_OFFS_USRH = 0x17,		// READ/ WRITE
        ZG_OFFS_USRL = 0x18,		// READ/ WRITE
        SMPLRT_DIV = 0x19,			// READ/ WRITE
        CONFIG = 0x1A,				// READ/ WRITE default value:0x80
        GYRO_CONFIG = 0x1B,			// READ/ WRITE
        ACCEL_CONFIG = 0x1C,		// READ/ WRITE
        ACCEL_CONFIG2 = 0x1D,		// READ/ WRITE
        LP_MODE_CFG = 0x1E,			// READ/ WRITE
        ACCEL_WOM_X_THR = 0x20,		// READ/ WRITE
        ACCEL_WOM_Y_THR = 0x21,		// READ/ WRITE
        ACCEL_WOM_Z_THR = 0x22,		// READ/ WRITE
        FIFO_EN = 0x23,				// READ/ WRITE
        FSYNC_INT = 0x36,			// READ to CLEAR
        INT_PIN_CFG = 0x37,			// READ/ WRITE
        INT_ENABLE = 0x38,			// READ/ WRITE
        FIFO_WM_INT_STATUS = 0x39,	// READ to CLEAR
        INT_STATUS = 0x3A,			// READ to CLEAR
        ACCEL_XOUT_H = 0x3B,		// READ
        ACCEL_XOUT_L = 0x3C,		// READ
        ACCEL_YOUT_H = 0x3D,		// READ
        ACCEL_YOUT_L = 0x3E,		// READ
        ACCEL_ZOUT_H = 0x3F,		// READ
        ACCEL_ZOUT_L = 0x40,		// READ
        TEMP_OUT_H = 0x41,			// READ
        TEMP_OUT_L = 0x42,			// READ
        GYRO_XOUT_H = 0x43,			// READ
        GYRO_XOUT_L = 0x44,			// READ
        GYRO_YOUT_H = 0x45,			// READ
        GYRO_YOUT_L = 0x46,			// READ
        GYRO_ZOUT_H = 0x47,			// READ
        GYRO_ZOUT_L = 0x48,			// READ
        SELF_TEST_X_GYRO = 0x50,	// READ/ WRITE
        SELF_TEST_Y_GYRO = 0x51,	// READ/ WRITE
        SELF_TEST_Z_GYRO = 0x52,	// READ/ WRITE
        FIFO_WM_TH1 = 0x60,			// READ/ WRITE
        FIFO_WM_TH2 = 0x61,			// READ/ WRITE
        SIGNAL_PATH_RESET = 0x68,	// READ/ WRITE
        ACCEL_INTEL_CTRL = 0x69,	// READ/ WRITE
        USER_CTRL = 0x6A,			// READ/ WRITE
        PWR_MGMT_1 = 0x6B,			// READ/ WRITE default value:0x41
        PWR_MGMT_2 = 0x6C,			// READ/ WRITE
        I2C_IF = 0x70,				// READ/ WRITE
        FIFO_COUNTH = 0x72,			// READ
        FIFO_COUNTL = 0x73,			// READ
        FIFO_R_W = 0x74,			// READ/ WRITE
        WHO_AM_I = 0x75,			// READ default value:0x12
        XA_OFFSET_H = 0x77,			// READ/ WRITE
        XA_OFFSET_L = 0x78,			// READ/ WRITE
        YA_OFFSET_H = 0x7A,			// READ/ WRITE
        YA_OFFSET_L = 0x7B,			// READ/ WRITE
        ZA_OFFSET_H = 0x7D,			// READ/ WRITE
        ZA_OFFSET_L = 0x7E,			// READ/ WRITE
    };


    enum class mpu6050_RegMap :uint8_t
    {
        SELF_TEST_X = 0x0D,			 //R/W
        SELF_TEST_Y = 0x0E,			 //R/W
        SELF_TEST_Z = 0x0F,			 //R/W
        SELF_TEST_A = 0x10,			 //R/W
        SMPLRT_DIV = 0x19,			 //R/W
        CONFIG = 0x1A,				 //R/W
        GYRO_CONFIG = 0x1B,			 //R/W
        ACCEL_CONFIG = 0x1C,		 //R/W
        FIFO_EN = 0x23,				 //R/W
        I2C_MST_CTRL = 0x24,		 //R/W
        I2C_SLV0_ADDR = 0x25,		 //R/W
        I2C_SLV0_REG = 0x26,		 //R/W
        I2C_SLV0_CTRL = 0x27,		 //R/W
        I2C_SLV1_ADDR = 0x28,		 //R/W
        I2C_SLV1_REG = 0x29,		 //R/W
        I2C_SLV1_CTRL = 0x2A,		 //R/W
        I2C_SLV2_ADDR = 0x2B,		 //R/W
        I2C_SLV2_REG = 0x2C,		 //R/W
        I2C_SLV2_CTRL = 0x2D,		 //R/W
        I2C_SLV3_ADDR = 0x2E,		 //R/W
        I2C_SLV3_REG = 0x2F,		 //R/W
        I2C_SLV3_CTRL = 0x30,		 //R/W
        I2C_SLV4_ADDR = 0x31,		 //R/W
        I2C_SLV4_REG = 0x32,		 //R/W
        I2C_SLV4_DO = 0x33,			 //R/W
        I2C_SLV4_CTRL = 0x34,		 //R/W
        I2C_SLV4_DI = 0x35,			 //R
        I2C_MST_STATUS = 0x36,		 //R
        INT_PIN_CFG = 0x37,			 //R/W
        INT_ENABLE = 0x38,			 //R/W
        INT_STATUS = 0x3A,			 //R
        ACCEL_XOUT_H = 0x3B,		 //R
        ACCEL_XOUT_L = 0x3C,		 //R
        ACCEL_YOUT_H = 0x3D,		 //R
        ACCEL_YOUT_L = 0x3E,		 //R
        ACCEL_ZOUT_H = 0x3F,		 //R
        ACCEL_ZOUT_L = 0x40,		 //R
        TEMP_OUT_H = 0x41,			 //R
        TEMP_OUT_L = 0x42,			 //R
        GYRO_XOUT_H = 0x43,			 //R
        GYRO_XOUT_L = 0x44,			 //R
        GYRO_YOUT_H = 0x45,			 //R
        GYRO_YOUT_L = 0x46,			 //R
        GYRO_ZOUT_H = 0x47,			 //R
        GYRO_ZOUT_L = 0x48,			 //R
        EXT_SENS_DATA_00 = 0x49,	 //R
        EXT_SENS_DATA_01 = 0x4A,	 //R
        EXT_SENS_DATA_02 = 0x4B,	 //R
        EXT_SENS_DATA_03 = 0x4C,	 //R
        EXT_SENS_DATA_04 = 0x4D,	 //R
        EXT_SENS_DATA_05 = 0x4E,	 //R
        EXT_SENS_DATA_06 = 0x4F,	 //R
        EXT_SENS_DATA_07 = 0x50,	 //R
        EXT_SENS_DATA_08 = 0x51,	 //R
        EXT_SENS_DATA_09 = 0x52,	 //R
        EXT_SENS_DATA_10 = 0x53,	 //R
        EXT_SENS_DATA_11 = 0x54,	 //R
        EXT_SENS_DATA_12 = 0x55,	 //R
        EXT_SENS_DATA_13 = 0x56,	 //R
        EXT_SENS_DATA_14 = 0x57,	 //R
        EXT_SENS_DATA_15 = 0x58,	 //R
        EXT_SENS_DATA_16 = 0x59,	 //R
        EXT_SENS_DATA_17 = 0x5A,	 //R
        EXT_SENS_DATA_18 = 0x5B,	 //R
        EXT_SENS_DATA_19 = 0x5C,	 //R
        EXT_SENS_DATA_20 = 0x5D,	 //R
        EXT_SENS_DATA_21 = 0x5E,	 //R
        EXT_SENS_DATA_22 = 0x5F,	 //R
        EXT_SENS_DATA_23 = 0x60,	 //R
        I2C_SLV0_DO = 0x63,			 //R/W
        I2C_SLV1_DO = 0x64,			 //R/W
        I2C_SLV2_DO = 0x65,			 //R/W
        I2C_SLV3_DO = 0x66,			 //R/W
        I2C_MST_DELAY_CTRL = 0x67,	 //R/W
        SIGNAL_PATH_RESET = 0x68,	 //R/W
        USER_CTRL = 0x6A,			 //R/W
        PWR_MGMT_1 = 0x6B,			 //R/W
        PWR_MGMT_2 = 0x6C,			 //R/W
        FIFO_COUNTH = 0x72,			 //R/W
        FIFO_COUNTL = 0x73,			 //R/W
        FIFO_R_W = 0x74,			 //R/W
        WHO_AM_I = 0x75,			 //R
    };

    enum class mpu9250_RegMap :uint8_t
    {
        SELF_TEST_X_GYRO = 0x0,//R/W
        SELF_TEST_Y_GYRO = 0x1,//R/W
        SELF_TEST_Z_GYRO = 0x2,//R/W
        SELF_TEST_X_ACCEL = 0x0D,//R/W
        SELF_TEST_Y_ACCEL = 0x0E,//R/W
        SELF_TEST_Z_ACCEL = 0x0F,//R/W
        XG_OFFSET_H = 0x13,//R/W
        XG_OFFSET_L = 0x14,//R/W
        YG_OFFSET_H = 0x15,//R/W
        YG_OFFSET_L = 0x16,//R/W
        ZG_OFFSET_H = 0x17,//R/W
        ZG_OFFSET_L = 0x18,//R/W
        SMPLRT_DIV = 0x19,//R/W
        CONFIG = 0x1A,//R/W
        GYRO_CONFIG = 0x1B,//R/W
        ACCEL_CONFIG = 0x1C,//R/W
        ACCEL_CONFIG2 = 0x1D,//R/W
        LP_ACCEL_ODR = 0x1E,//R/W
        WOM_THR = 0x1F,//R/W
        FIFO_EN = 0x23,//R/W
        I2C_MST_CTRL = 0x24,//R/W
        I2C_SLV0_ADDR = 0x25,//R/W
        I2C_SLV0_REG = 0x26,//R/W
        I2C_SLV0_CTRL = 0x27,//R/W
        I2C_SLV1_ADDR = 0x28,//R/W
        I2C_SLV1_REG = 0x29,//R/W
        I2C_SLV1_CTRL = 0x2A,//R/W
        I2C_SLV2_ADDR = 0x2B,//R/W
        I2C_SLV2_REG = 0x2C,//R/W
        I2C_SLV2_CTRL = 0x2D,//R/W
        I2C_SLV3_ADDR = 0x2E,//R/W
        I2C_SLV3_REG = 0x2F,//R/W
        I2C_SLV3_CTRL = 0x30,//R/W
        I2C_SLV4_ADDR = 0x31,//R/W
        I2C_SLV4_REG = 0x32,//R/W
        I2C_SLV4_DO = 0x33,//R/W
        I2C_SLV4_CTRL = 0x34,//R/W
        I2C_SLV4_DI = 0x35,//R
        I2C_MST_STATUS = 0x36,//R
        INT_PIN_CFG = 0x37,//R/W
        INT_ENABLE = 0x38,//R/W
        INT_STATUS = 0x3A,//R
        ACCEL_XOUT_H = 0x3B,//R
        ACCEL_XOUT_L = 0x3C,//R
        ACCEL_YOUT_H = 0x3D,//R
        ACCEL_YOUT_L = 0x3E,//R
        ACCEL_ZOUT_H = 0x3F,//R
        ACCEL_ZOUT_L = 0x40,//R
        TEMP_OUT_H = 0x41,//R
        TEMP_OUT_L = 0x42,//R
        GYRO_XOUT_H = 0x43,//R
        GYRO_XOUT_L = 0x44,//R
        GYRO_YOUT_H = 0x45,//R
        GYRO_YOUT_L = 0x46,//R
        GYRO_ZOUT_H = 0x47,//R
        GYRO_ZOUT_L = 0x48,//R
        EXT_SENS_DATA_00 = 0x49,//R
        EXT_SENS_DATA_01 = 0x4A,//R
        EXT_SENS_DATA_02 = 0x4B,//R
        EXT_SENS_DATA_03 = 0x4C,//R
        EXT_SENS_DATA_04 = 0x4D,//R
        EXT_SENS_DATA_05 = 0x4E,//R
        EXT_SENS_DATA_06 = 0x4F,//R
        EXT_SENS_DATA_07 = 0x50,//R
        EXT_SENS_DATA_08 = 0x51,//R
        EXT_SENS_DATA_09 = 0x52,//R
        EXT_SENS_DATA_10 = 0x53,//R
        EXT_SENS_DATA_11 = 0x54,//R
        EXT_SENS_DATA_12 = 0x55,//R
        EXT_SENS_DATA_13 = 0x56,//R
        EXT_SENS_DATA_14 = 0x57,//R
        EXT_SENS_DATA_15 = 0x58,//R
        EXT_SENS_DATA_16 = 0x59,//R
        EXT_SENS_DATA_17 = 0x5A,//R
        EXT_SENS_DATA_18 = 0x5B,//R
        EXT_SENS_DATA_19 = 0x5C,//R
        EXT_SENS_DATA_20 = 0x5D,//R
        EXT_SENS_DATA_21 = 0x5E,//R
        EXT_SENS_DATA_22 = 0x5F,//R
        EXT_SENS_DATA_23 = 0x60,//R
        I2C_SLV0_DO = 0x63,//R/W
        I2C_SLV1_DO = 0x64,//R/W
        I2C_SLV2_DO = 0x65,//R/W
        I2C_SLV3_DO = 0x66,//R/W
        I2C_MST_DELAY_CTRL = 0x67,//R/W
        SIGNAL_PATH_RESET = 0x68,//R/W
        MOT_DETECT_CTRL = 0x69,//R/W
        USER_CTRL = 0x6A,//R/W
        PWR_MGMT_1 = 0x6B,//R/W
        PWR_MGMT_2 = 0x6C,//R/W
        FIFO_COUNTH = 0x72,//R/W
        FIFO_COUNTL = 0x73,//R/W
        FIFO_R_W = 0x74,//R/W
        WHO_AM_I = 0x75,//R
        XA_OFFSET_H = 0x77,//R/W
        XA_OFFSET_L = 0x78,//R/W
        YA_OFFSET_H = 0x7A,//R/W
        YA_OFFSET_L = 0x7B,//R/W
        ZA_OFFSET_H = 0x7D,//R/W
        ZA_OFFSET_L = 0x7E,//R/W
    };

    enum class ak8963_RegMap : uint8_t {
        //Magnetometer register maps
        WIA = 0x00,
        INFO = 0x01,
        ST1 = 0x02,
        XOUT_L = 0x03,
        XOUT_H = 0x04,
        YOUT_L = 0x05,
        YOUT_H = 0x06,
        ZOUT_L = 0x07,
        ZOUT_H = 0x08,
        ST2 = 0x09,
        CNTL = 0x0A,
        CNTL2 = 0x0B,
        RSV = 0x0B, //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
        ASTC = 0x0C,
        TS1 = 0x0D, //DO NOT ACCESS
        TS2 = 0x0E, //DO NOT ACCESS
        I2CDIS = 0x0F,
        ASAX = 0x10,
        ASAY = 0x11,
        ASAZ = 0x12,
    };

    enum class ak09916_RegMap : uint8_t {
        //Magnetometer register maps
        WIA2 = 0x01,
        ST1 = 0x10,
        XOUT_L = 0x11,
        XOUT_H = 0x12,
        YOUT_L = 0x13,
        YOUT_H = 0x14,
        ZOUT_L = 0x15,
        ZOUT_H = 0x16,
        ST2 = 0x18,
        CNTL2 = 0x31,
        CNTL3 = 0x32,
        TS1 = 0x33, //DO NOT ACCESS
        TS2 = 0x34, //DO NOT ACCESS
    };

    enum class icm20948_RegMap:uint16_t{
        WHO_AM_I                    =0x0       ,//0      R
        USER_CTRL                   =0x3       ,//3      R/W
        LP_CONFIG                   =0x5       ,//5      R/W
        PWR_MGMT_1                  =0x6       ,//6      R/W
        PWR_MGMT_2                  =0x7       ,//7      R/W
        INT_PIN_CFG                 =0x0F      ,//15     R/W
        INT_ENABLE                  =0x10      ,//16     R/W
        INT_ENABLE_1                =0x11      ,//17     R/W
        INT_ENABLE_2                =0x12      ,//18     R/W
        INT_ENABLE_3                =0x13      ,//19     R/W
        I2C_MST_STATUS              =0x17      ,//23     R/C
        INT_STATUS                  =0x19      ,//25     R/C
        INT_STATUS_1                =0x1A      ,//26     R/C
        INT_STATUS_2                =0x1B      ,//27     R/C
        INT_STATUS_3                =0x1C      ,//28     R/C
        DELAY_TIMEH                 =0x28      ,//40     R
        DELAY_TIMEL                 =0x29      ,//41     R
        ACCEL_XOUT_H                =0x2D      ,//45     R
        ACCEL_XOUT_L                =0x2E      ,//46     R
        ACCEL_YOUT_H                =0x2F      ,//47     R
        ACCEL_YOUT_L                =0x30      ,//48     R
        ACCEL_ZOUT_H                =0x31      ,//49     R
        ACCEL_ZOUT_L                =0x32      ,//50     R
        GYRO_XOUT_H                 =0x33      ,//51     R
        GYRO_XOUT_L                 =0x34      ,//52     R
        GYRO_YOUT_H                 =0x35      ,//53     R
        GYRO_YOUT_L                 =0x36      ,//54     R
        GYRO_ZOUT_H                 =0x37      ,//55     R
        GYRO_ZOUT_L                 =0x38      ,//56     R
        TEMP_OUT_H                  =0x39      ,//57     R
        TEMP_OUT_L                  =0x3A      ,//58     R
        EXT_SLV_SENS_DATA_00        =0x3B      ,//59     R
        EXT_SLV_SENS_DATA_01        =0x3C      ,//60     R
        EXT_SLV_SENS_DATA_02        =0x3D      ,//61     R
        EXT_SLV_SENS_DATA_03        =0x3E      ,//62     R
        EXT_SLV_SENS_DATA_04        =0x3F      ,//63     R
        EXT_SLV_SENS_DATA_05        =0x40      ,//64     R
        EXT_SLV_SENS_DATA_06        =0x41      ,//65     R
        EXT_SLV_SENS_DATA_07        =0x42      ,//66     R
        EXT_SLV_SENS_DATA_08        =0x43      ,//67     R
        EXT_SLV_SENS_DATA_09        =0x44      ,//68     R
        EXT_SLV_SENS_DATA_10        =0x45      ,//69     R
        EXT_SLV_SENS_DATA_11        =0x46      ,//70     R
        EXT_SLV_SENS_DATA_12        =0x47      ,//71     R
        EXT_SLV_SENS_DATA_13        =0x48      ,//72     R
        EXT_SLV_SENS_DATA_14        =0x49      ,//73     R
        EXT_SLV_SENS_DATA_15        =0x4A      ,//74     R
        EXT_SLV_SENS_DATA_16        =0x4B      ,//75     R
        EXT_SLV_SENS_DATA_17        =0x4C      ,//76     R
        EXT_SLV_SENS_DATA_18        =0x4D      ,//77     R
        EXT_SLV_SENS_DATA_19        =0x4E      ,//78     R
        EXT_SLV_SENS_DATA_20        =0x4F      ,//79     R
        EXT_SLV_SENS_DATA_21        =0x50      ,//80     R
        EXT_SLV_SENS_DATA_22        =0x51      ,//81     R
        EXT_SLV_SENS_DATA_23        =0x52      ,//82     R
        FIFO_EN_1                   =0x66      ,//102    R/W
        FIFO_EN_2                   =0x67      ,//103    R/W
        FIFO_RST                    =0x68      ,//104    R/W
        FIFO_MODE                   =0x69      ,//105    R/W
        FIFO_COUNTH                 =0x70      ,//112    R
        FIFO_COUNTL                 =0x71      ,//113    R
        FIFO_R_W                    =0x72      ,//114    R/W
        DATA_RDY_STATUS             =0x74      ,//116    R/C
        FIFO_CFG                    =0x76      ,//118    R/W
        REG_BANK_SEL                =0x7F      ,//127    R/W


        SELF_TEST_X_GYRO             =(1<<8)|0x2      ,//2     R/W
        SELF_TEST_Y_GYRO             =(1<<8)|0x3      ,//3     R/W
        SELF_TEST_Z_GYRO             =(1<<8)|0x4      ,//4     R/W
        SELF_TEST_X_ACCEL            =(1<<8)|0x0E     ,//14    R/W
        SELF_TEST_Y_ACCEL            =(1<<8)|0x0F     ,//15    R/W
        SELF_TEST_Z_ACCEL            =(1<<8)|0x10     ,//16    R/W
        XA_OFFS_H                    =(1<<8)|0x14     ,//20    R/W
        XA_OFFS_L                    =(1<<8)|0x15     ,//21    R/W
        YA_OFFS_H                    =(1<<8)|0x17     ,//23    R/W
        YA_OFFS_L                    =(1<<8)|0x18     ,//24    R/W
        ZA_OFFS_H                    =(1<<8)|0x1A     ,//26    R/W
        ZA_OFFS_L                    =(1<<8)|0x1B     ,//27    R/W
        TIMEBASE_CORRECTION_PLL      =(1<<8)|0x28     ,//40    R/W
//        REG_BANK_SEL                 =(1<<8)|0x7F     ,//127   R/W



        GYRO_SMPLRT_DIV             =(2<<8)|0x0          ,//0       R/W
        GYRO_CONFIG_1               =(2<<8)|0x1          ,//1       R/W
        GYRO_CONFIG_2               =(2<<8)|0x2          ,//2       R/W
        XG_OFFS_USRH                =(2<<8)|0x3          ,//3       R/W
        XG_OFFS_USRL                =(2<<8)|0x4          ,//4       R/W
        YG_OFFS_USRH                =(2<<8)|0x5          ,//5       R/W
        YG_OFFS_USRL                =(2<<8)|0x6          ,//6       R/W
        ZG_OFFS_USRH                =(2<<8)|0x7          ,//7       R/W
        ZG_OFFS_USRL                =(2<<8)|0x8          ,//8       R/W
        ODR_ALIGN_EN                =(2<<8)|0x9          ,//9       R/W
        ACCEL_SMPLRT_DIV_1          =(2<<8)|0x10         ,//16      R/W
        ACCEL_SMPLRT_DIV_2          =(2<<8)|0x11         ,//17      R/W
        ACCEL_INTEL_CTRL            =(2<<8)|0x12         ,//18      R/W
        ACCEL_WOM_THR               =(2<<8)|0x13         ,//19      R/W
        ACCEL_CONFIG                =(2<<8)|0x14         ,//20      R/W
        ACCEL_CONFIG_2              =(2<<8)|0x15         ,//21      R/W
        FSYNC_CONFIG                =(2<<8)|0x52         ,//82      R/W
        TEMP_CONFIG                 =(2<<8)|0x53         ,//83      R/W
        MOD_CTRL_USR                =(2<<8)|0x54         ,//84      R/W
//        REG_BANK_SEL                =(2<<8)|0x7F         ,//127     R/W



        I2C_MST_ODR_CONFIG        =(3<<8)|0x0       ,//0      R/W
        I2C_MST_CTRL              =(3<<8)|0x1       ,//1      R/W
        I2C_MST_DELAY_CTRL        =(3<<8)|0x2       ,//2      R/W
        I2C_SLV0_ADDR             =(3<<8)|0x3       ,//3      R/W
        I2C_SLV0_REG              =(3<<8)|0x4       ,//4      R/W
        I2C_SLV0_CTRL             =(3<<8)|0x5       ,//5      R/W
        I2C_SLV0_DO               =(3<<8)|0x6       ,//6      R/W
        I2C_SLV1_ADDR             =(3<<8)|0x7       ,//7      R/W
        I2C_SLV1_REG              =(3<<8)|0x8       ,//8      R/W
        I2C_SLV1_CTRL             =(3<<8)|0x9       ,//9      R/W
        I2C_SLV1_DO               =(3<<8)|0x0A      ,//10     R/W
        I2C_SLV2_ADDR             =(3<<8)|0x0B      ,//11     R/W
        I2C_SLV2_REG              =(3<<8)|0x0C      ,//12     R/W
        I2C_SLV2_CTRL             =(3<<8)|0x0D      ,//13     R/W
        I2C_SLV2_DO               =(3<<8)|0x0E      ,//14     R/W
        I2C_SLV3_ADDR             =(3<<8)|0x0F      ,//15     R/W
        I2C_SLV3_REG              =(3<<8)|0x10      ,//16     R/W
        I2C_SLV3_CTRL             =(3<<8)|0x11      ,//17     R/W
        I2C_SLV3_DO               =(3<<8)|0x12      ,//18     R/W
        I2C_SLV4_ADDR             =(3<<8)|0x13      ,//19     R/W
        I2C_SLV4_REG              =(3<<8)|0x14      ,//20     R/W
        I2C_SLV4_CTRL             =(3<<8)|0x15      ,//21     R/W
        I2C_SLV4_DO               =(3<<8)|0x16      ,//22     R/W
        I2C_SLV4_DI               =(3<<8)|0x17      ,//23     R
//        REG_BANK_SEL              =(3<<8)|0x7F      ,//127    R/W

    };

    constexpr static const int DEF_ST_PRECISION = 1000;
    constexpr static const int DEF_GYRO_CT_SHIFT_DELTA = 500;
    constexpr static const int DEF_ACCEL_ST_SHIFT_DELTA = 500;
    /* Gyro Offset Max Value (dps) */
    constexpr static const int DEF_GYRO_OFFSET_MAX = 20;
    /* Gyro Self Test Absolute Limits ST_AL (dps) */
    constexpr static const int DEF_GYRO_ST_AL = 60;
    /* Accel Self Test Absolute Limits ST_AL (mg) */
    constexpr static const int DEF_ACCEL_ST_AL_MIN = 225;
    constexpr static const int DEF_ACCEL_ST_AL_MAX = 675;

    constexpr static const uint16_t sSelfTestEquation[256] = {
            2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
            2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
            3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
            3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
            3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
            3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
            4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
            4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
            4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
            5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
            5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
            6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
            6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
            7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
            7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
            8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
            9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
            10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
            10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
            11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
            12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
            13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
            15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
            16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
            17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
            19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
            20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
            22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
            24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
            26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
            28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
            30903, 31212, 31524, 31839, 32157, 32479, 32804
    };

    constexpr static const uint16_t accelSelfTestEquation[32] = {
            1347, 1393, 1440, 1488, 1538, 1590, 1644, 1699,
            1757, 1816, 1877, 1941, 2006, 2074, 2144, 2216,
            2291, 2368, 2448, 2531, 2616, 2704, 2795, 2890,
            2987, 3088, 3192, 3300, 3411, 3526, 3645, 3768,
    };
    constexpr static const uint16_t gyroSelfTestEquation[32] = {
            3131, 3275, 3426, 3583, 3748, 3920, 4101, 4289,
            4487, 4693, 4909, 5135, 5371, 5618, 5877, 6147,
            6430, 6725, 7035, 7358, 7697, 8051, 8421, 8809,
            9214, 9638, 10081, 10545, 11030, 11537, 12068, 12623};

    constexpr static const int MPU9250_I2C_SLV4_EN = 0x80;
    constexpr static const int MPU9250_I2C_SLV4_DONE = 0x40;
    constexpr static const int MPU9250_I2C_SLV4_NACK = 0x10;
    constexpr static const int MPU9250_AK8963_I2C_ADDR = 0x0C;
    constexpr static const int ICM20948_AK09916_I2C_ADDR = 0x0C;
    constexpr static const int MPU9250_AK8963_POWER_DOWN = 0x10;
    constexpr static const int MPU9250_AK8963_FUSE_ROM_ACCESS = 0x1F;
    constexpr static const int MPU9250_AK8963_SINGLE_MEASUREMENT = 0x11;
    constexpr static const int MPU9250_AK8963_CONTINUOUS_MEASUREMENT = 0x16; //MODE 2
    constexpr static const int MPU9250_AK8963_DATA_READY = (0x01);
    constexpr static const int MPU9250_AK8963_DATA_OVERRUN = (0x02);
    //constexpr static const int MPU9250_AK8963_OVERFLOW = (0x80);
    constexpr static const int MPU9250_AK8963_OVERFLOW = (0x08);
    constexpr static const int MPU9250_AK8963_DATA_ERROR = (0x40);
    constexpr static const int MPU9250_AK8963_CNTL2_SRST = 0x01;
#endif
}
#endif //UTILITIES_DRV_IMU_INVENSENSE_HPP
