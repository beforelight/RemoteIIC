#ifndef _INV_IMU_HPP_
#define _INV_IMU_HPP_

#include <cstdint>
#include <string>
#include <memory>
#include <functional>
#include <utility>
#include <vector>
#if defined(__linux__) && !defined(INV_PRINTF)
#include<cstdio>
#define INV_PRINTF printf
#else
#define INV_PRINTF(...)
#endif //!defined(INV_PRINTF)

#ifdef INV_YES_TRACE
#define INV_TRACE_(fmt, ...) \
    INV_PRINTF("%s:%d:trace: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_TRACE(...) INV_TRACE_(__VA_ARGS__, "")
#else
#define INV_TRACE(...)
#endif //INV_YES_TRACE

#ifndef INV_NO_DEBUG
#define INV_DEBUG_(fmt, ...) \
    INV_PRINTF("%s:%d:debug: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) INV_DEBUG_(__VA_ARGS__, "")
#else
#define INV_DEBUG(...)
#endif //INV_NO_DEBUG

namespace inv {
    class I2C {
    public:
        struct Transfer {
            Transfer() : slaveAddress(0), slaveAddressSize(1), subAddressSize(1), subAddress(0), data(nullptr), dataSize(0), direction(Write) {}
            uint16_t slaveAddress;
            uint8_t slaveAddressSize;
            uint8_t subAddressSize;
            uint32_t subAddress;
            void *data;
            uint32_t dataSize;
            enum Direction {
                Write = 0U, /*!< Master transmit. */
                Read = 1U  /*!< Master receive. */
            } direction;
        };
        std::function<int(const Transfer &)> masterTransferBlocking;
        std::function<int(const Transfer &)> masterTransferNonBlocking;
        I2C(std::function<int(const Transfer &)> _masterTransferBlocking,
            std::function<int(const Transfer &)> _masterTransferNonBlocking)
                : masterTransferBlocking(std::move(_masterTransferBlocking)),
                  masterTransferNonBlocking(std::move(_masterTransferNonBlocking)) {}
        explicit I2C(const std::function<int(const Transfer &)> &_masterTransferBlocking)
                : I2C(_masterTransferBlocking, _masterTransferBlocking) {}
    };

    class SPI {
    public:
        struct Transfer {
            Transfer() : txData(nullptr), rxData(nullptr), dataSize(0) {}
            uint8_t *txData;          /*!< Send buffer. */
            uint8_t *rxData;          /*!< Receive buffer. */
            volatile uint32_t dataSize; /*!< Transfer bytes. */
        };
        std::function<int(const Transfer &)> masterTransferBlocking;
        std::function<int(const Transfer &)> masterTransferNonBlocking;
        SPI(std::function<int(const Transfer &)> _masterTransferBlocking,
            std::function<int(const Transfer &)> _masterTransferNonBlocking)
                : masterTransferBlocking(std::move(_masterTransferBlocking)),
                  masterTransferNonBlocking(std::move(_masterTransferNonBlocking)) {}
        explicit SPI(const std::function<int(const Transfer &)> &_masterTransferBlocking)
                : SPI(_masterTransferBlocking, _masterTransferBlocking) {}
    };

    struct Config {
        enum mpu_accel_fs {
            MPU_FS_2G = 2,
            MPU_FS_4G = 4,
            MPU_FS_8G = 8,
            MPU_FS_16G = 16,
        } accelFullScale;

        enum mpu_accel_bw {
            MPU_ABW_420 = 420,
            MPU_ABW_218 = 218,
            MPU_ABW_99 = 99,
            MPU_ABW_45 = 45,
            MPU_ABW_21 = 21,
            MPU_ABW_10 = 10,
            MPU_ABW_5 = 5,
        } accelBandwidth;

        enum mpu_gyro_fs {
            MPU_FS_125dps = 125,
            MPU_FS_250dps = 250,
            MPU_FS_500dps = 500,
            MPU_FS_1000dps = 1000,
            MPU_FS_2000dps = 2000,
        } gyroFullScale;

        enum mpu_gyro_bw {
            MPU_GBW_361 = 361,
            MPU_GBW_250 = 250,
            MPU_GBW_176 = 176,
            MPU_GBW_92 = 92,
            MPU_GBW_41 = 41,
            MPU_GBW_20 = 20,
            MPU_GBW_10 = 10,
            MPU_GBW_5 = 5,
        } gyroBandwidth;

        enum mpu_gyro_unit {
            MPU_UNIT_DegPerSec = 0,
            MPU_UNIT_RadPerSec,
            MPU_UNIT_RevolutionsPerMinute,
        } gyroUnit;

        enum mpu_accel_unit {
            MPU_UNIT_MetersPerSquareSecond = 0,
            MPU_UNIT_G,
            MPU_UNIT_mG
        } accelUnit;

        explicit Config(mpu_accel_fs _accel_fs = MPU_FS_8G, mpu_accel_bw _accel_bw = MPU_ABW_99,
                        mpu_accel_unit mpuAccelUnit = MPU_UNIT_MetersPerSquareSecond,
                        mpu_gyro_fs _gyro_gs = MPU_FS_2000dps, mpu_gyro_bw _gyro_bw = MPU_GBW_92,
                        mpu_gyro_unit mpuGyroUnit = MPU_UNIT_DegPerSec)
                : accelFullScale(_accel_fs), accelBandwidth(_accel_bw),
                  gyroFullScale(_gyro_gs), gyroBandwidth(_gyro_bw), gyroUnit(mpuGyroUnit), accelUnit(mpuAccelUnit) {}
    };


    class IMU {
    public:
        virtual ~IMU() {}
        virtual int Init(Config _cfg = Config()) = 0;
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
        virtual bool IsOpen();
    public:
        static constexpr const uint16_t SlaveAddressAutoDetect = 0;
        explicit IMU(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect);
        explicit IMU(SPI &_spi);
        int WriteReg(uint8_t reg, uint8_t val);
        int WriteRegVerified(uint8_t reg, uint8_t val);
        int ReadReg(uint8_t reg, uint8_t *val);
        int ModifyReg(uint8_t reg, uint8_t val, uint8_t mask);
    protected:
        I2C *i2c;
        SPI *spi;
        I2C::Transfer i2cTransfer;
        SPI::Transfer spiTransfer;
        bool addrAutoDetect;
        bool isOpen;
        Config cfg;
    };

    class MPU6050 : public IMU {
    public:
        ~MPU6050() {}
        MPU6050(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : IMU(_i2c, _addr) {}

        int Init(Config _cfg = Config()) override;
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
    public:
        enum class MPU6050_RegMap : uint8_t {
            SELF_TEST_X = 0x0D,             //R/W
            SELF_TEST_Y = 0x0E,             //R/W
            SELF_TEST_Z = 0x0F,             //R/W
            SELF_TEST_A = 0x10,             //R/W
            SMPLRT_DIV = 0x19,             //R/W
            CONFIG = 0x1A,                 //R/W
            GYRO_CONFIG = 0x1B,             //R/W
            ACCEL_CONFIG = 0x1C,         //R/W
            FIFO_EN = 0x23,                 //R/W
            I2C_MST_CTRL = 0x24,         //R/W
            I2C_SLV0_ADDR = 0x25,         //R/W
            I2C_SLV0_REG = 0x26,         //R/W
            I2C_SLV0_CTRL = 0x27,         //R/W
            I2C_SLV1_ADDR = 0x28,         //R/W
            I2C_SLV1_REG = 0x29,         //R/W
            I2C_SLV1_CTRL = 0x2A,         //R/W
            I2C_SLV2_ADDR = 0x2B,         //R/W
            I2C_SLV2_REG = 0x2C,         //R/W
            I2C_SLV2_CTRL = 0x2D,         //R/W
            I2C_SLV3_ADDR = 0x2E,         //R/W
            I2C_SLV3_REG = 0x2F,         //R/W
            I2C_SLV3_CTRL = 0x30,         //R/W
            I2C_SLV4_ADDR = 0x31,         //R/W
            I2C_SLV4_REG = 0x32,         //R/W
            I2C_SLV4_DO = 0x33,             //R/W
            I2C_SLV4_CTRL = 0x34,         //R/W
            I2C_SLV4_DI = 0x35,             //R
            I2C_MST_STATUS = 0x36,         //R
            INT_PIN_CFG = 0x37,             //R/W
            INT_ENABLE = 0x38,             //R/W
            INT_STATUS = 0x3A,             //R
            ACCEL_XOUT_H = 0x3B,         //R
            ACCEL_XOUT_L = 0x3C,         //R
            ACCEL_YOUT_H = 0x3D,         //R
            ACCEL_YOUT_L = 0x3E,         //R
            ACCEL_ZOUT_H = 0x3F,         //R
            ACCEL_ZOUT_L = 0x40,         //R
            TEMP_OUT_H = 0x41,             //R
            TEMP_OUT_L = 0x42,             //R
            GYRO_XOUT_H = 0x43,             //R
            GYRO_XOUT_L = 0x44,             //R
            GYRO_YOUT_H = 0x45,             //R
            GYRO_YOUT_L = 0x46,             //R
            GYRO_ZOUT_H = 0x47,             //R
            GYRO_ZOUT_L = 0x48,             //R
            EXT_SENS_DATA_00 = 0x49,     //R
            EXT_SENS_DATA_01 = 0x4A,     //R
            EXT_SENS_DATA_02 = 0x4B,     //R
            EXT_SENS_DATA_03 = 0x4C,     //R
            EXT_SENS_DATA_04 = 0x4D,     //R
            EXT_SENS_DATA_05 = 0x4E,     //R
            EXT_SENS_DATA_06 = 0x4F,     //R
            EXT_SENS_DATA_07 = 0x50,     //R
            EXT_SENS_DATA_08 = 0x51,     //R
            EXT_SENS_DATA_09 = 0x52,     //R
            EXT_SENS_DATA_10 = 0x53,     //R
            EXT_SENS_DATA_11 = 0x54,     //R
            EXT_SENS_DATA_12 = 0x55,     //R
            EXT_SENS_DATA_13 = 0x56,     //R
            EXT_SENS_DATA_14 = 0x57,     //R
            EXT_SENS_DATA_15 = 0x58,     //R
            EXT_SENS_DATA_16 = 0x59,     //R
            EXT_SENS_DATA_17 = 0x5A,     //R
            EXT_SENS_DATA_18 = 0x5B,     //R
            EXT_SENS_DATA_19 = 0x5C,     //R
            EXT_SENS_DATA_20 = 0x5D,     //R
            EXT_SENS_DATA_21 = 0x5E,     //R
            EXT_SENS_DATA_22 = 0x5F,     //R
            EXT_SENS_DATA_23 = 0x60,     //R
            I2C_SLV0_DO = 0x63,             //R/W
            I2C_SLV1_DO = 0x64,             //R/W
            I2C_SLV2_DO = 0x65,             //R/W
            I2C_SLV3_DO = 0x66,             //R/W
            I2C_MST_DELAY_CTRL = 0x67,     //R/W
            SIGNAL_PATH_RESET = 0x68,     //R/W
            USER_CTRL = 0x6A,             //R/W
            PWR_MGMT_1 = 0x6B,             //R/W
            PWR_MGMT_2 = 0x6C,             //R/W
            FIFO_COUNTH = 0x72,             //R/W
            FIFO_COUNTL = 0x73,             //R/W
            FIFO_R_W = 0x74,             //R/W
            WHO_AM_I = 0x75,             //R
        };
    };

    class ICM20602 : public IMU {
    public:
        virtual ~ICM20602() {}
        ICM20602(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : IMU(_i2c, _addr), buf(rxbuf + 1) {}
        ICM20602(SPI &_spi) : IMU(_spi), buf(rxbuf + 1) {}
        int Init(Config _cfg = Config()) override;
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
        uint8_t *buf;
        uint8_t txbuf[15];
        uint8_t rxbuf[15];
    public:
        enum class ICM20602_RegMap : uint8_t {
            XG_OFFS_TC_H = 0x4,            // READ/ WRITE
            XG_OFFS_TC_L = 0x5,            // READ/ WRITE
            YG_OFFS_TC_H = 0x7,            // READ/ WRITE
            YG_OFFS_TC_L = 0x8,            // READ/ WRITE
            ZG_OFFS_TC_H = 0x0A,        // READ/ WRITE
            ZG_OFFS_TC_L = 0x0B,        // READ/ WRITE
            SELF_TEST_X_ACCEL = 0x0D,    // READ/ WRITE
            SELF_TEST_Y_ACCEL = 0x0E,    // READ/ WRITE
            SELF_TEST_Z_ACCEL = 0x0F,    // READ/ WRITE
            XG_OFFS_USRH = 0x13,        // READ/ WRITE
            XG_OFFS_USRL = 0x14,        // READ/ WRITE
            YG_OFFS_USRH = 0x15,        // READ/ WRITE
            YG_OFFS_USRL = 0x16,        // READ/ WRITE
            ZG_OFFS_USRH = 0x17,        // READ/ WRITE
            ZG_OFFS_USRL = 0x18,        // READ/ WRITE
            SMPLRT_DIV = 0x19,            // READ/ WRITE
            CONFIG = 0x1A,                // READ/ WRITE default value:0x80
            GYRO_CONFIG = 0x1B,            // READ/ WRITE
            ACCEL_CONFIG = 0x1C,        // READ/ WRITE
            ACCEL_CONFIG2 = 0x1D,        // READ/ WRITE
            LP_MODE_CFG = 0x1E,            // READ/ WRITE
            ACCEL_WOM_X_THR = 0x20,        // READ/ WRITE
            ACCEL_WOM_Y_THR = 0x21,        // READ/ WRITE
            ACCEL_WOM_Z_THR = 0x22,        // READ/ WRITE
            FIFO_EN = 0x23,                // READ/ WRITE
            FSYNC_INT = 0x36,            // READ to CLEAR
            INT_PIN_CFG = 0x37,            // READ/ WRITE
            INT_ENABLE = 0x38,            // READ/ WRITE
            FIFO_WM_INT_STATUS = 0x39,    // READ to CLEAR
            INT_STATUS = 0x3A,            // READ to CLEAR
            ACCEL_XOUT_H = 0x3B,        // READ
            ACCEL_XOUT_L = 0x3C,        // READ
            ACCEL_YOUT_H = 0x3D,        // READ
            ACCEL_YOUT_L = 0x3E,        // READ
            ACCEL_ZOUT_H = 0x3F,        // READ
            ACCEL_ZOUT_L = 0x40,        // READ
            TEMP_OUT_H = 0x41,            // READ
            TEMP_OUT_L = 0x42,            // READ
            GYRO_XOUT_H = 0x43,            // READ
            GYRO_XOUT_L = 0x44,            // READ
            GYRO_YOUT_H = 0x45,            // READ
            GYRO_YOUT_L = 0x46,            // READ
            GYRO_ZOUT_H = 0x47,            // READ
            GYRO_ZOUT_L = 0x48,            // READ
            SELF_TEST_X_GYRO = 0x50,    // READ/ WRITE
            SELF_TEST_Y_GYRO = 0x51,    // READ/ WRITE
            SELF_TEST_Z_GYRO = 0x52,    // READ/ WRITE
            FIFO_WM_TH1 = 0x60,            // READ/ WRITE
            FIFO_WM_TH2 = 0x61,            // READ/ WRITE
            SIGNAL_PATH_RESET = 0x68,    // READ/ WRITE
            ACCEL_INTEL_CTRL = 0x69,    // READ/ WRITE
            USER_CTRL = 0x6A,            // READ/ WRITE
            PWR_MGMT_1 = 0x6B,            // READ/ WRITE default value:0x41
            PWR_MGMT_2 = 0x6C,            // READ/ WRITE
            I2C_IF = 0x70,                // READ/ WRITE
            FIFO_COUNTH = 0x72,            // READ
            FIFO_COUNTL = 0x73,            // READ
            FIFO_R_W = 0x74,            // READ/ WRITE
            WHO_AM_I = 0x75,            // READ default value:0x12
            XA_OFFSET_H = 0x77,            // READ/ WRITE
            XA_OFFSET_L = 0x78,            // READ/ WRITE
            YA_OFFSET_H = 0x7A,            // READ/ WRITE
            YA_OFFSET_L = 0x7B,            // READ/ WRITE
            ZA_OFFSET_H = 0x7D,            // READ/ WRITE
            ZA_OFFSET_L = 0x7E,            // READ/ WRITE
        };
    };

    class ICM20600 : public ICM20602 {
    public:
    public:
        virtual ~ICM20600() {}
        ICM20600(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : ICM20602(_i2c, _addr) {}
        ICM20600(SPI &_spi) : ICM20602(_spi) {}
        bool Detect() override;
        std::string Report() override;
    };

    class MPU9250 : public IMU {
    public:
        virtual ~MPU9250() {}

        MPU9250(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : IMU(_i2c, _addr), buf(rxbuf + 1) {}
        MPU9250(SPI &_spi) : IMU(_spi), buf(rxbuf + 1) {}

        int Init(Config _cfg = Config()) override;
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
        uint8_t *buf;
        uint8_t txbuf[23];
        uint8_t rxbuf[23];
        float gyroUnit;
        float accelUnit;
        uint8_t ak8963DeviceId;
        uint8_t ak8963Information;
        constexpr static float magUnit = 0.15f;;//固定量程4900uT 0.15µT/LSB
        float ak8963Asa[3];
    public:
        enum class MPU9250_RegMap : uint8_t {
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
    };

    class ICM20948 : public IMU {
    public:
        ~ICM20948() {}
        ICM20948(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : IMU(_i2c, _addr), bank(0), buf(rxbuf + 1) {}
        ICM20948(SPI &_spi) : IMU(_spi), bank(0), buf(rxbuf + 1) {}
        int Init(Config _cfg = Config()) override;
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
        uint8_t *buf;
        uint8_t txbuf[24];
        uint8_t rxbuf[24];
        constexpr static float magUnit = 0.15f;;//固定量程4900uT 0.15µT/LSB
    public:
        enum class ICM20948_RegMap : uint16_t {
            WHO_AM_I = 0x0,//0      R
            USER_CTRL = 0x3,//3      R/W
            LP_CONFIG = 0x5,//5      R/W
            PWR_MGMT_1 = 0x6,//6      R/W
            PWR_MGMT_2 = 0x7,//7      R/W
            INT_PIN_CFG = 0x0F,//15     R/W
            INT_ENABLE = 0x10,//16     R/W
            INT_ENABLE_1 = 0x11,//17     R/W
            INT_ENABLE_2 = 0x12,//18     R/W
            INT_ENABLE_3 = 0x13,//19     R/W
            I2C_MST_STATUS = 0x17,//23     R/C
            INT_STATUS = 0x19,//25     R/C
            INT_STATUS_1 = 0x1A,//26     R/C
            INT_STATUS_2 = 0x1B,//27     R/C
            INT_STATUS_3 = 0x1C,//28     R/C
            DELAY_TIMEH = 0x28,//40     R
            DELAY_TIMEL = 0x29,//41     R
            ACCEL_XOUT_H = 0x2D,//45     R
            ACCEL_XOUT_L = 0x2E,//46     R
            ACCEL_YOUT_H = 0x2F,//47     R
            ACCEL_YOUT_L = 0x30,//48     R
            ACCEL_ZOUT_H = 0x31,//49     R
            ACCEL_ZOUT_L = 0x32,//50     R
            GYRO_XOUT_H = 0x33,//51     R
            GYRO_XOUT_L = 0x34,//52     R
            GYRO_YOUT_H = 0x35,//53     R
            GYRO_YOUT_L = 0x36,//54     R
            GYRO_ZOUT_H = 0x37,//55     R
            GYRO_ZOUT_L = 0x38,//56     R
            TEMP_OUT_H = 0x39,//57     R
            TEMP_OUT_L = 0x3A,//58     R
            EXT_SLV_SENS_DATA_00 = 0x3B,//59     R
            EXT_SLV_SENS_DATA_01 = 0x3C,//60     R
            EXT_SLV_SENS_DATA_02 = 0x3D,//61     R
            EXT_SLV_SENS_DATA_03 = 0x3E,//62     R
            EXT_SLV_SENS_DATA_04 = 0x3F,//63     R
            EXT_SLV_SENS_DATA_05 = 0x40,//64     R
            EXT_SLV_SENS_DATA_06 = 0x41,//65     R
            EXT_SLV_SENS_DATA_07 = 0x42,//66     R
            EXT_SLV_SENS_DATA_08 = 0x43,//67     R
            EXT_SLV_SENS_DATA_09 = 0x44,//68     R
            EXT_SLV_SENS_DATA_10 = 0x45,//69     R
            EXT_SLV_SENS_DATA_11 = 0x46,//70     R
            EXT_SLV_SENS_DATA_12 = 0x47,//71     R
            EXT_SLV_SENS_DATA_13 = 0x48,//72     R
            EXT_SLV_SENS_DATA_14 = 0x49,//73     R
            EXT_SLV_SENS_DATA_15 = 0x4A,//74     R
            EXT_SLV_SENS_DATA_16 = 0x4B,//75     R
            EXT_SLV_SENS_DATA_17 = 0x4C,//76     R
            EXT_SLV_SENS_DATA_18 = 0x4D,//77     R
            EXT_SLV_SENS_DATA_19 = 0x4E,//78     R
            EXT_SLV_SENS_DATA_20 = 0x4F,//79     R
            EXT_SLV_SENS_DATA_21 = 0x50,//80     R
            EXT_SLV_SENS_DATA_22 = 0x51,//81     R
            EXT_SLV_SENS_DATA_23 = 0x52,//82     R
            FIFO_EN_1 = 0x66,//102    R/W
            FIFO_EN_2 = 0x67,//103    R/W
            FIFO_RST = 0x68,//104    R/W
            FIFO_MODE = 0x69,//105    R/W
            FIFO_COUNTH = 0x70,//112    R
            FIFO_COUNTL = 0x71,//113    R
            FIFO_R_W = 0x72,//114    R/W
            DATA_RDY_STATUS = 0x74,//116    R/C
            FIFO_CFG = 0x76,//118    R/W
            REG_BANK_SEL = 0x7F,//127    R/W


            SELF_TEST_X_GYRO = (1 << 8) | 0x2,//2     R/W
            SELF_TEST_Y_GYRO = (1 << 8) | 0x3,//3     R/W
            SELF_TEST_Z_GYRO = (1 << 8) | 0x4,//4     R/W
            SELF_TEST_X_ACCEL = (1 << 8) | 0x0E,//14    R/W
            SELF_TEST_Y_ACCEL = (1 << 8) | 0x0F,//15    R/W
            SELF_TEST_Z_ACCEL = (1 << 8) | 0x10,//16    R/W
            XA_OFFS_H = (1 << 8) | 0x14,//20    R/W
            XA_OFFS_L = (1 << 8) | 0x15,//21    R/W
            YA_OFFS_H = (1 << 8) | 0x17,//23    R/W
            YA_OFFS_L = (1 << 8) | 0x18,//24    R/W
            ZA_OFFS_H = (1 << 8) | 0x1A,//26    R/W
            ZA_OFFS_L = (1 << 8) | 0x1B,//27    R/W
            TIMEBASE_CORRECTION_PLL = (1 << 8) | 0x28,//40    R/W
//        REG_BANK_SEL                 =(1<<8)|0x7F     ,//127   R/W



            GYRO_SMPLRT_DIV = (2 << 8) | 0x0,//0       R/W
            GYRO_CONFIG_1 = (2 << 8) | 0x1,//1       R/W
            GYRO_CONFIG_2 = (2 << 8) | 0x2,//2       R/W
            XG_OFFS_USRH = (2 << 8) | 0x3,//3       R/W
            XG_OFFS_USRL = (2 << 8) | 0x4,//4       R/W
            YG_OFFS_USRH = (2 << 8) | 0x5,//5       R/W
            YG_OFFS_USRL = (2 << 8) | 0x6,//6       R/W
            ZG_OFFS_USRH = (2 << 8) | 0x7,//7       R/W
            ZG_OFFS_USRL = (2 << 8) | 0x8,//8       R/W
            ODR_ALIGN_EN = (2 << 8) | 0x9,//9       R/W
            ACCEL_SMPLRT_DIV_1 = (2 << 8) | 0x10,//16      R/W
            ACCEL_SMPLRT_DIV_2 = (2 << 8) | 0x11,//17      R/W
            ACCEL_INTEL_CTRL = (2 << 8) | 0x12,//18      R/W
            ACCEL_WOM_THR = (2 << 8) | 0x13,//19      R/W
            ACCEL_CONFIG = (2 << 8) | 0x14,//20      R/W
            ACCEL_CONFIG_2 = (2 << 8) | 0x15,//21      R/W
            FSYNC_CONFIG = (2 << 8) | 0x52,//82      R/W
            TEMP_CONFIG = (2 << 8) | 0x53,//83      R/W
            MOD_CTRL_USR = (2 << 8) | 0x54,//84      R/W
//        REG_BANK_SEL                =(2<<8)|0x7F         ,//127     R/W



            I2C_MST_ODR_CONFIG = (3 << 8) | 0x0,//0      R/W
            I2C_MST_CTRL = (3 << 8) | 0x1,//1      R/W
            I2C_MST_DELAY_CTRL = (3 << 8) | 0x2,//2      R/W
            I2C_SLV0_ADDR = (3 << 8) | 0x3,//3      R/W
            I2C_SLV0_REG = (3 << 8) | 0x4,//4      R/W
            I2C_SLV0_CTRL = (3 << 8) | 0x5,//5      R/W
            I2C_SLV0_DO = (3 << 8) | 0x6,//6      R/W
            I2C_SLV1_ADDR = (3 << 8) | 0x7,//7      R/W
            I2C_SLV1_REG = (3 << 8) | 0x8,//8      R/W
            I2C_SLV1_CTRL = (3 << 8) | 0x9,//9      R/W
            I2C_SLV1_DO = (3 << 8) | 0x0A,//10     R/W
            I2C_SLV2_ADDR = (3 << 8) | 0x0B,//11     R/W
            I2C_SLV2_REG = (3 << 8) | 0x0C,//12     R/W
            I2C_SLV2_CTRL = (3 << 8) | 0x0D,//13     R/W
            I2C_SLV2_DO = (3 << 8) | 0x0E,//14     R/W
            I2C_SLV3_ADDR = (3 << 8) | 0x0F,//15     R/W
            I2C_SLV3_REG = (3 << 8) | 0x10,//16     R/W
            I2C_SLV3_CTRL = (3 << 8) | 0x11,//17     R/W
            I2C_SLV3_DO = (3 << 8) | 0x12,//18     R/W
            I2C_SLV4_ADDR = (3 << 8) | 0x13,//19     R/W
            I2C_SLV4_REG = (3 << 8) | 0x14,//20     R/W
            I2C_SLV4_CTRL = (3 << 8) | 0x15,//21     R/W
            I2C_SLV4_DO = (3 << 8) | 0x16,//22     R/W
            I2C_SLV4_DI = (3 << 8) | 0x17,//23     R
//        REG_BANK_SEL              =(3<<8)|0x7F      ,//127    R/W

        };
    };


    class QMC5883L : public IMU {
    public:
        ~QMC5883L() {}
        QMC5883L(I2C &_i2c) : IMU(_i2c, 0x0D) {}
        int Init(Config _cfg = Config()) override;
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
        uint8_t buf[9];
        constexpr static float magUnit = 0.0244140625f;//固定量程800uT/32768.0
    public:
        enum class QMC5883L_RegMap : uint8_t {
            XOUT_L = 0x00,
            XOUT_H = 0x01,
            YOUT_L = 0x02,
            YOUT_H = 0x03,
            ZOUT_L = 0x04,
            ZOUT_H = 0x05,
            STATUS = 0x06,
            TOUT_L = 0x07,
            TOUT_H = 0x08,
            CONTROL1 = 0x09,
            CONTROL2 = 0x0A,
            SET_RESET_PERIOD = 0x0B //必须设置为0x01
        };
    };

//    class BMX160 : public IMU {
//    public:
//        ~BMX160() {}
//        BMX160(I2C &_i2c, uint16_t _addr = SlaveAddressAutoDetect) : IMU(_i2c, _addr) {}
//        BMX160(SPI &_spi) : IMU(_spi) {}
//        int Init(Config _cfg = Config()) override;
//        bool Detect() override;
//        int SelfTest() override;
//        std::string Report() override;
//        bool DataReady() override;
//        int EnableDataReadyInt() override;
//        int SoftReset() override;
//        int ReadSensorBlocking() override;
//        int ReadSensorNonBlocking() override;
//        int Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) override;
//        int Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) override;
//        int Convert(float *mag_x, float *mag_y, float *mag_z) override;
//        int Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) override;
//        int Convert(float *temp) override;
//    };


    class IMU_Ptr : public std::shared_ptr<IMU> {
    public:
        int Load(I2C &_i2c, uint16_t _addr = IMU::SlaveAddressAutoDetect);
        int Load(SPI &_spi);
    };

#if 1

    enum class AK8963_RegMap : uint8_t {
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

    enum class AK09916_RegMap : uint8_t {
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

    enum class BMX160_RegMap : uint8_t {
        CHIP_ID = 0x00,  ///<Chip Identification.
        ERR_REG = 0x02,  ///<Reports sensor error flags.  Flags reset when read.
        PMU_STATUS = 0x03,      ///<Reports current power mode for sensors.
        DATA_0 = 0x04,          ///<MAG_X axis bits7:0
        DATA_1 = 0x05,          ///<MAG_X axis bits15:8
        DATA_2 = 0x06,          ///<MAG_Y axis bits7:0
        DATA_3 = 0x07,          ///<MAG_Y axis bits15:8
        DATA_4 = 0x08,          ///<MAG_Z axis bits7:0
        DATA_5 = 0x09,          ///<MAG_Z axis bits15:8
        DATA_6 = 0x0A,          ///<RHALL bits7:0
        DATA_7 = 0x0B,          ///<RHALL bits15:8
        DATA_8 = 0x0C,          ///<GYR_X axis bits7:0
        DATA_9 = 0x0D,          ///<GYR_X axis bits15:8
        DATA_10 = 0x0E,         ///<GYR_Y axis bits7:0
        DATA_11 = 0x0F,         ///<GYR_Y axis bits15:8
        DATA_12 = 0x10,         ///<GYR_Z axis bits7:0
        DATA_13 = 0x11,         ///<GYR_Z axis bits15:8
        DATA_14 = 0x12,         ///<ACC_X axis bits7:0
        DATA_15 = 0x13,         ///<ACC_X axis bits15:8
        DATA_16 = 0x14,         ///<ACC_Y axis bits7:0
        DATA_17 = 0x15,         ///<ACC_Y axis bits15:8
        DATA_18 = 0x16,         ///<ACC_Z axis bits7:0
        DATA_19 = 0x17,         ///<ACC_Z axis bits15:8
        SENSORTIME_0 = 0x18,     ///<24bit counter synchronized with data, bits7:0
        SENSORTIME_1 = 0x19,    ///<24bit counter synchronized with data, bits15:8
        SENSORTIME_2 = 0x1A,    ///<24bit counter synchronized with data, bits23:16
        STATUS = 0x1B,          ///<Reports sensors status flags
        INT_STATUS_0 = 0x1C,    ///<Contains interrupt status flags
        INT_STATUS_1 = 0x1D,    ///<Contains interrupt status flags
        INT_STATUS_2 = 0x1E,    ///<Contains interrupt status flags
        INT_STATUS_3 = 0x1F,    ///<Contains interrupt status flags
        TEMPERATURE_0 = 0x20,   ///<Contains temperature of sensor, bits7:0
        TEMPERATURE_1 = 0x21,   ///<Contains temperature of sensor, bits15:8
        FIFO_LENGTH_0 = 0x22,   ///<Current fill level of FIFO, bits7:0
        FIFO_LENGTH_1 = 0x23,   ///<Current fill level of FIFO, bits10:8
        FIFO_DATA = 0x24,       ///<FIFO data read out register, burst read
        ACC_CONF = 0x40, ///<Set ODR, bandwidth, and read mode of accelerometer
        ACC_RANGE = 0x41,       ///<Sets accelerometer g-range
        GYR_CONF = 0x42,        ///<Set ODR, bandwidth, and read mode of gyroscope
        GYR_RANGE = 0x43,       ///<Sets gyroscope angular rate measurement range
        MAG_CONF = 0x44,        ///<Sets ODR of magnetometer interface
        FIFO_DOWNS = 0x45,      ///<Sets down sampling ratios of accel and gyro data
        ///<for FIFO
        FIFO_CONFIG_0 = 0x46,   ///<Sets FIFO Watermark
        FIFO_CONFIG_1 = 0x47,   ///<Sets which sensor data is available in FIFO,
        ///<Header/Headerless mode, Ext Int tagging, Sensortime
        MAG_IF_0 = 0x4c, ///<Magnetometer 7-bit I2C address, bits7:1
        MAG_IF_1 = 0x4D,        ///<Magnetometer interface configuration
        MAG_IF_2 = 0x4E,        ///<Magnetometer address to read
        MAG_IF_3 = 0x4F,        ///<Magnetometer address to write
        INT_EN_0 = 0x50,        ///<Interrupt enable bits
        INT_EN_1 = 0x51,        ///<Interrupt enable bits
        INT_EN_2 = 0x52,        ///<Interrupt enable bits
        INT_OUT_CTRL = 0x53,    ///<Contains the behavioral configuration of INT pins
        INT_LATCH = 0x54,       ///<Contains the interrupt rest bit and the interrupt
        ///<mode selection
        INT_MAP_0 = 0x55,       ///<Controls which interrupt signals are mapped to the
        ///<INT1 and INT2 pins
        INT_MAP_1 = 0x56,       ///<Controls which interrupt signals are mapped to the
        ///<INT1 and INT2 pins
        INT_MAP_2 = 0x57,       ///<Controls which interrupt signals are mapped to the
        ///<INT1 and INT2 pins
        INT_DATA_0 = 0x58,      ///<Contains the data source definition for the two
        ///<interrupt groups
        INT_DATA_1 = 0x59,      ///<Contains the data source definition for the two
        ///<interrupt groups
        INT_LOWHIGH_0 = 0x5A,   ///<Contains the configuration for the low g interrupt
        INT_LOWHIGH_1 = 0x5B,   ///<Contains the configuration for the low g interrupt
        INT_LOWHIGH_2 = 0x5C,   ///<Contains the configuration for the low g interrupt
        INT_LOWHIGH_3 = 0x5D,   ///<Contains the configuration for the low g interrupt
        INT_LOWHIGH_4 = 0x5E,   ///<Contains the configuration for the low g interrupt
        INT_MOTION_0 = 0x5F,    ///<Contains the configuration for the any motion and
        ///<no motion interrupts
        INT_MOTION_1 = 0x60,    ///<Contains the configuration for the any motion and
        ///<no motion interrupts
        INT_MOTION_2 = 0x61,    ///<Contains the configuration for the any motion and
        ///<no motion interrupts
        INT_MOTION_3 = 0x62,    ///<Contains the configuration for the any motion and
        ///<no motion interrupts
        INT_TAP_0 = 0x63,       ///<Contains the configuration for the tap interrupts
        INT_TAP_1 = 0x64,       ///<Contains the configuration for the tap interrupts
        INT_ORIENT_0 = 0x65,    ///<Contains the configuration for the oeientation
        ///<interrupt
        INT_ORIENT_1 = 0x66,    ///<Contains the configuration for the oeientation
        ///<interrupt
        INT_FLAT_0 = 0x67,      ///<Contains the configuration for the flat interrupt
        INT_FLAT_1 = 0x68,      ///<Contains the configuration for the flat interrupt
        FOC_CONF = 0x69,        ///<Contains configuration for the fast offset
        ///<compensation for the accelerometer and gyroscope
        CONF = 0x6A,            ///<Configuration of sensor, nvm_prog_en bit
        IF_CONF = 0x6B,         ///<Contains settings for the digital interface
        PMU_TRIGGER = 0x6C,     ///<Sets trigger conditions to change gyro power modes
        SELF_TEST = 0x6D,       ///<Self test configuration
        NV_CONF = 0x70,  ///<Contains settings for the digital interface
        OFFSET_0 = 0x71,        ///<Contains offset comp values for acc_off_x7:0
        OFFSET_1 = 0x72,        ///<Contains offset comp values for acc_off_y7:0
        OFFSET_2 = 0x73,        ///<Contains offset comp values for acc_off_z7:0
        OFFSET_3 = 0x74,        ///<Contains offset comp values for gyr_off_x7:0
        OFFSET_4 = 0x75,        ///<Contains offset comp values for gyr_off_y7:0
        OFFSET_5 = 0x76,        ///<Contains offset comp values for gyr_off_z7:0
        OFFSET_6 = 0x77,        ///<gyr/acc offset enable bit and gyr_off_(zyx) bits9:8
        STEP_CNT_0 = 0x78,      ///<Step counter bits 15:8
        STEP_CNT_1 = 0x79,      ///<Step counter bits 7:0
        STEP_CONF_0 = 0x7A,     ///<Contains configuration of the step detector
        STEP_CONF_1 = 0x7B,     ///<Contains configuration of the step detector
        CMD = 0x7E       ///<Command register triggers operations like
        ///<softreset, NVM programming, etc.
    };

#endif //1
}
#endif //_INV_IMU_HPP_
