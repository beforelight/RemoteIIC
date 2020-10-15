//
// Created by 17616 on 2020/10/14.
//
#include <cstring>
#include <cmath>
#include"drv_imu_invensense.hpp"

bool inv::icm20602_t::Detect() {
    uint8_t val;
    SetI2cAddr(0x68);
    if (0 != ReadReg((uint8_t) icm20602_RegMap::WHO_AM_I, &val)) { return false; };
    if (0x12 == val) { return true; }
    SetI2cAddr(0x69);
    if (0 != ReadReg((uint8_t) icm20602_RegMap::WHO_AM_I, &val)) { return false; };
    if (0x12 == val) { return true; }
    return false;
}

std::string inv::icm20602_t::Report() {
    std::string rtv;
    rtv += "model:icm20602\t";
    rtv += "addr:";
    rtv += std::to_string((int) GetI2cAddr());
    rtv += '\t';
    return rtv;
}

int inv::mpu6500Series_t::SelfTest() {
    if (!IsOpen()) { return -1; }
    int res = 0;
    config_t backup_cfg = GetConfig();
    config_t st_cfg;
    st_cfg.gyroFullScale = config_t::MPU_FS_250dps;
    st_cfg.accelFullScale = config_t::MPU_FS_2G;
    st_cfg.accelBandwidth = config_t::MPU_ABW_99;
    st_cfg.gyroBandwidth = config_t::MPU_GBW_92;
    if (0 != Init(st_cfg)) {
        Init(backup_cfg);
        return -1;
    }
    int32_t gyro_bias_st[3], gyro_bias_regular[3];
    int32_t accel_bias_st[3], accel_bias_regular[3];
    int16_t abuf[3];
    int16_t gbuf[3];
    int accel_result = 0;
    int gyro_result = 0;
    uint8_t val;
    memset(gyro_bias_st, 0, sizeof(gyro_bias_st));
    memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
    memset(accel_bias_st, 0, sizeof(accel_bias_st));
    memset(accel_bias_regular, 0, sizeof(accel_bias_regular));

    int times;
    times = 20;
    while (times--) { while (!DataReady()) {}}//丢弃前20个数据
    times = 20;
    while (times--) {
        while (!DataReady()) {}
        res |= ReadSensorBlocking();
        Converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] += gbuf[i];
            accel_bias_regular[i] += abuf[i];
        }
    }

    res |= ReadReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, &val);
    res |= WriteReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
    res |= ReadReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, &val);
    res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
    times = 20;
    while (times--) { while (!DataReady()) {}}//丢弃前20个数据
    times = 20;
    while (times--) {
        while (!DataReady()) {}
        res |= ReadSensorBlocking();
        Converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_st[i] += gbuf[i];
            accel_bias_st[i] += abuf[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        gyro_bias_regular[i] *= 50;   //(32768/2000)*1000 LSB/mg
        accel_bias_regular[i] *= 50;
        gyro_bias_st[i] *= 50;         //(32768/250)*1000 LSB/dps
        accel_bias_st[i] *= 50;
    }


    //计算加速度计自检结果
    uint8_t regs[3];
    int otp_value_zero = 0;
    int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
//    int result;

    res |= i2c.ReadBlocking(GetI2cAddr(), REG_SELF_TEST_X_ACCEL(), regs, 3);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    if (!otp_value_zero) {
        /* Self Test Pass/Fail Criteria A */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = accel_bias_st[i] - accel_bias_regular[i];
            st_shift_ratio[i] = abs(st_shift_cust[i] / st_shift_prod[i] - DEF_ST_PRECISION);
            if (st_shift_ratio[i] > DEF_ACCEL_ST_SHIFT_DELTA) {
                //加速度计自检未通过
                accel_result = 1;
            }
        }
    } else {
        /* Self Test Pass/Fail Criteria B */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = abs(accel_bias_st[i] - accel_bias_regular[i]);
            if (st_shift_cust[i] < DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000
                || st_shift_cust[i] > DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000) {
                //加速度计自检未通过
                accel_result = 1;
            }
        }
    }

    //计算陀螺仪自检结果
    res |= i2c.ReadBlocking(GetI2cAddr(), REG_SELF_TEST_X_GYRO(), regs, 3);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    for (i = 0; i < 3; i++) {
        st_shift_cust[i] = gyro_bias_st[i] - gyro_bias_regular[i];
        if (!otp_value_zero) {
            /* Self Test Pass/Fail Criteria A */
            if (st_shift_cust[i] < DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]) {
                //陀螺仪自检没过
                gyro_result = 1;
            }
        } else {
            /* Self Test Pass/Fail Criteria B */
            if (st_shift_cust[i] < DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION) {
                //陀螺仪自检没过
                gyro_result = 1;
            }
        }
    }

    if (gyro_result == 0) {
        /* Self Test Pass/Fail Criteria C */
        for (i = 0; i < 3; i++) {
            if (abs(gyro_bias_regular[i]) > DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION)
                //陀螺仪自检没过
            {
                gyro_result = 1;
            }
        }
    }

    //恢复原来的配置
    res |= Init(backup_cfg);
    return res | (gyro_result << 1) | accel_result;
}

namespace inv {


    int mpuxxxx_t::Init(config_t _cfg) {
        SetConfig(_cfg);
        ClearIsOpen();
        int res = 0;
        if (!Detect()) { return -1; }
        //软复位
        res |= SoftReset();

        //打开所有传感器
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_2, 0);

        //1khz采样率
        res |= WriteReg((uint8_t) icm20602_RegMap::SMPLRT_DIV, 0);

        //配置陀螺仪lpf
        switch (GetConfig().gyroBandwidth) {
            case config_t::MPU_GBW_250:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 0);
                break;
            case config_t::MPU_GBW_176:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 1);
                break;
            case config_t::MPU_GBW_92:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 2);
                break;
            case config_t::MPU_GBW_41:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 3);
                break;
            case config_t::MPU_GBW_20:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 4);
                break;
            case config_t::MPU_GBW_10:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 5);
                break;
            case config_t::MPU_GBW_5:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 6);
                break;
            default:
                res |= WriteReg((uint8_t) icm20602_RegMap::CONFIG, 0);
                break;
        }

        //配置陀螺仪量程
        switch (GetConfig().gyroFullScale) {
            case config_t::MPU_FS_250dps:
                res |= WriteReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 0 << 3);
                gyroUnit = 250.0 / 32768;
                break;
            case config_t::MPU_FS_500dps:
                res |= WriteReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 1 << 3);
                gyroUnit = 500.0 / 32768;
                break;
            case config_t::MPU_FS_1000dps:
                res |= WriteReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 2 << 3);
                gyroUnit = 1000.0 / 32768;
                break;
            case config_t::MPU_FS_2000dps:
            default:
                res |= WriteReg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 3 << 3);
                gyroUnit = 2000.0 / 32768;
                break;
        }

        //配置加速度计量程
        switch (GetConfig().accelFullScale) {
            case config_t::MPU_FS_2G:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 0 << 3);
                accelUnit = 2.0 * 9.8 / 32768;
                break;
            case config_t::MPU_FS_4G:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 1 << 3);
                accelUnit = 4.0 * 9.8 / 32768;
                break;
            case config_t::MPU_FS_8G:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 2 << 3);
                accelUnit = 8.0 * 9.8 / 32768;
                break;
            case config_t::MPU_FS_16G:
            default:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 3 << 3);
                accelUnit = 16.0 * 9.8 / 32768;
                break;
        }

        //配置加速度计lpf
        switch (GetConfig().accelBandwidth) {
            case config_t::MPU_ABW_218:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 1);
                break;
            case config_t::MPU_ABW_99:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 2);
                break;
            case config_t::MPU_ABW_45:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 3);
                break;
            case config_t::MPU_ABW_21:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 4);
                break;
            case config_t::MPU_ABW_10:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 5);
                break;
            case config_t::MPU_ABW_5:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 6);
                break;
            case config_t::MPU_ABW_420:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 7);
                break;
            default:
                res |= WriteReg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 1);
                break;
        }

        //开启数据更新中断
        res |= EnableDataReadyInt();

        if (res == 0) {
            SetIsOpen();
        }
        return res;
    }

    int mpuxxxx_t::Converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x,
                             int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int mpuxxxx_t::Converter(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y,
                             float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int icm20602_t::Converter(float *temp) {
        if (temp) { *temp = (float) ((int16_t) (buf[6] << 8) | buf[7]) / 326.8f + 25.0f; }
        return 0;
    }

    int mpuxxxx_t::ReadSensorBlocking() {
        return i2c.ReadBlocking(GetI2cAddr(), (uint8_t) icm20602_RegMap::ACCEL_XOUT_H, buf, 14);
    }

    int mpuxxxx_t::ReadSensorNonBlocking() {
        return i2c.ReadNonBlocking(GetI2cAddr(),
                                   (uint8_t) icm20602_RegMap::ACCEL_XOUT_H,
                                   buf, 14);
    }

    int icm20602_t::SoftReset(void) {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= ReadReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x41);

        //唤起睡眠
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x1);

        return res;
    }

    bool mpuxxxx_t::DataReady() {
        uint8_t val = 0;
        ReadReg((uint8_t) icm20602_RegMap::INT_STATUS, &val);
        return (val & 0x01) == 0x01;
    }

    mpuxxxx_t::mpuxxxx_t(i2cInterface_t &_i2c) : imu_t(_i2c), accelUnit(0), gyroUnit(0) {
        memset(buf, 0, sizeof(buf));
    }

    int mpuxxxx_t::Converter(float *mag_x, float *mag_y, float *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    int mpuxxxx_t::Converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    int mpuxxxx_t::EnableDataReadyInt() {
        //enables interrupt generation by DATA_RDY
        return ModifyReg((uint8_t) icm20602_RegMap::INT_ENABLE, 0x01, 0x01);
    }

    bool mpu6050_t::Detect() {
        uint8_t val;
        SetI2cAddr(0x68);
        if (0 != ReadReg((uint8_t) mpu6050_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x68 == val) {
            return true;
        }
        SetI2cAddr(0x69);
        if (0 != ReadReg((uint8_t) mpu6050_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x68 == val) {
            return true;
        }
        return false;
    }

    int mpu6050_t::SelfTest() {
        if (!IsOpen()) { return -1; }
        int res = 0;
        config_t backup_cfg = GetConfig();
        config_t st_cfg;
        st_cfg.gyroFullScale = config_t::MPU_FS_250dps;
        st_cfg.accelFullScale = config_t::MPU_FS_8G;
        if (0 != Init(st_cfg)) {
            Init(backup_cfg);
            return -1;
        }
        int32_t gyro_bias_st[3], gyro_bias_regular[3];
        int32_t accel_bias_st[3], accel_bias_regular[3];
        int16_t abuf[3];
        int16_t gbuf[3];
        int accel_result = 0;
        int gyro_result = 0;
        uint8_t val;
        memset(gyro_bias_st, 0, sizeof(gyro_bias_st));
        memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
        memset(accel_bias_st, 0, sizeof(accel_bias_st));
        memset(accel_bias_regular, 0, sizeof(accel_bias_regular));


        int times;
        times = 20;
        while (times--) { while (!DataReady()) {}}//丢弃前20个数据
        times = 20;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            mpuxxxx_t::Converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        res |= ReadReg((uint8_t) mpu6050_RegMap::GYRO_CONFIG, &val);
        res |= WriteReg((uint8_t) mpu6050_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= ReadReg((uint8_t) mpu6050_RegMap::ACCEL_CONFIG, &val);
        res |= WriteReg((uint8_t) mpu6050_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        times = 20;
        while (times--) { while (!DataReady()) {}}//丢弃前100个数据
        times = 20;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            mpuxxxx_t::Converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_st[i] += gbuf[i];
                accel_bias_st[i] += abuf[i];
            }
        }

        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] *= 50;   //(32768/2000)*1000 LSB/mg
            accel_bias_regular[i] *= 50;
            gyro_bias_st[i] *= 50;         //(32768/250)*1000 LSB/dps
            accel_bias_st[i] *= 50;
        }

        //开始计算自检结果
        uint8_t regs[4];
        res |= i2c.ReadBlocking(GetI2cAddr(), (uint8_t) mpu6050_RegMap::SELF_TEST_X, regs, 4);
        int a_st[3];
        int g_st[3];
        float ft_a[3];
        float ft_g[3];
        a_st[0] = ((0b111 & (regs[0] >> 5)) << 2) | (0b11 & (regs[3] >> 4));
        a_st[1] = ((0b111 & (regs[1] >> 5)) << 2) | (0b11 & (regs[3] >> 2));
        a_st[2] = ((0b111 & (regs[2] >> 5)) << 2) | (0b11 & (regs[3] >> 0));
        g_st[0] = 0b11111 & regs[0];
        g_st[1] = 0b11111 & regs[1];
        g_st[2] = 0b11111 & regs[2];

        ft_a[0] = (a_st[0] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (float) (a_st[0] - 1) / ((1 << 5) - 2));
        ft_a[1] = (a_st[1] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (float) (a_st[1] - 1) / ((1 << 5) - 2));
        ft_a[2] = (a_st[2] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (float) (a_st[2] - 1) / ((1 << 5) - 2));
        ft_g[0] = (g_st[0] == 0) ? 0 : 1000 * 25 * 131 * pow(1.046, (float) g_st[0] - 1);
        ft_g[1] = (g_st[1] == 0) ? 0 : 1000 * -25 * 131 * pow(1.046, (float) g_st[1] - 1);
        ft_g[2] = (g_st[2] == 0) ? 0 : 1000 * 25 * 131 * pow(1.046, (float) g_st[2] - 1);

        for (int i = 0; i < 3; ++i) {
            int str = accel_bias_st[i] - accel_bias_regular[i];
            float Change_from_factory_trim = (float) (str - ft_a[i]) / ft_a[i];
            if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
                accel_result = 1;
            }
        }

        for (int i = 0; i < 3; ++i) {
            int str = gyro_bias_st[i] - gyro_bias_regular[i];
            float Change_from_factory_trim = (float) (str - ft_g[i]) / ft_g[i];
            if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
                gyro_result = 1;
            }
        }

        //恢复原来的配置
        res |= Init(backup_cfg);
        return (gyro_result << 1) | accel_result | res;
    }


    int mpu6050_t::Converter(float *temp) {
        if (temp) {
            *temp = (float) ((int16_t) (mpuxxxx_t::buf[6] << 8)
                             | mpuxxxx_t::buf[7] - 521) / 340.0f + 35;
        }
        return 0;
    }

    std::string mpu6050_t::Report() {
        std::string rtv;
        rtv += "model:mpu6050\t";
        rtv += "addr:";
        rtv += std::to_string((int) GetI2cAddr());
        rtv += '\t';
        return rtv;
    }

    int mpu6050_t::SoftReset(void) {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= ReadReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x40);

        //唤起睡眠
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x0);

        return res;
    }


    int mpu9250_t::init(config_t _cfg) {
        int res = mpuxxxx_t::Init((_cfg));
        if (res != 0) { return res; }
        ClearIsOpen();

        uint8_t val;
        //设置9250内部i2c
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_MST_CTRL, 1 << 4 | 9);//500khz，连续读模式
        res |= WriteReg((uint8_t) mpu9250_RegMap::USER_CTRL, 1 << 5);//开启i2c主模式

        //开始设置ak8963
        //读取id
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::WIA, &ak8963DeviceId, 1);
        if (res != 0) { return res; }
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::INFO, &ak8963Information, 1);

        //复位并且校准磁力计
        val = MPU9250_AK8963_CNTL2_SRST;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL2, &val, 1);
        val = MPU9250_AK8963_POWER_DOWN;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);
        val = MPU9250_AK8963_FUSE_ROM_ACCESS;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);
        //AK8963 get calibration data
        uint8_t response[3] = {0, 0, 0};
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::ASAX, response, 3);
        //AK8963_SENSITIVITY_SCALE_FACTOR
        //ak8963Asa[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
        //ak8963Asa[i++] = (s16)((data - 128.0f) *0.00390625f + 1.0f) ;
        ak8963Asa[0] = (1.0f + 0.00390625f * ((int16_t) (response[0]) - 128));
        ak8963Asa[1] = (1.0f + 0.00390625f * ((int16_t) (response[1]) - 128));
        ak8963Asa[2] = (1.0f + 0.00390625f * ((int16_t) (response[2]) - 128));

        val = MPU9250_AK8963_POWER_DOWN;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);

        //设置连续读ak8963到fifo
        val = 0x5D;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_MST_CTRL, val);

        val = MPU9250_AK8963_I2C_ADDR | 0x80;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV0_ADDR, val);

        val = (uint8_t) (uint8_t) ak8963_RegMap::ST1;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV0_REG, val);

        val = 0x88;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV0_CTRL, val);

        val = MPU9250_AK8963_CONTINUOUS_MEASUREMENT;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);

        val = 0x09;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, val);

        val = 0x81;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_MST_DELAY_CTRL, val);

        if (res == 0) {
            SetIsOpen();
            return 0;
        } else {
            return res;
        }
    }

    std::string mpu9250_t::Report() {
        std::string rtv;
        rtv += "model:mpu9250\t";
        rtv += "addr:";
        rtv += std::to_string((int) GetI2cAddr());
        rtv += '\t';

        rtv += "magnetometer:ak8963\t";
        rtv += "ID:";
        rtv += std::to_string((int) ak8963DeviceId);
        rtv += '\t';
        rtv += "INF:";
        rtv += std::to_string((int) ak8963Information);
        rtv += '\t';
        return rtv;
    }

    int mpu9250_t::SubI2cRead(unsigned char addr, unsigned char reg, unsigned char *val,
                              unsigned int len) {
        uint8_t index = 0;
        uint8_t status = 0;
        uint32_t timeout = 0;
        uint8_t tmp = 0;
        int res = 0;
        tmp = addr | 0x80;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    return -2;
                }
                res |= ReadReg((uint8_t) mpu9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            res |= ReadReg((uint8_t) mpu9250_RegMap::I2C_SLV4_DI, val + index);
            index++;
        }
        return res;
    }

    int mpu9250_t::SubI2cWrite(unsigned char addr, unsigned char reg, const unsigned char *val,
                               unsigned int len) {
        uint32_t timeout = 0;
        uint8_t status = 0;
        uint8_t tmp = 0;
        uint8_t index = 0;
        int res = 0;
        tmp = addr;
        res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = val[index];
            res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_DO, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    return -2;
                }
                res |= ReadReg((uint8_t) mpu9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            if (status & MPU9250_I2C_SLV4_NACK) {
                return -3;
            }
            index++;
        }
        return res;
    }

    bool mpu9250_t::Detect() {
        uint8_t val;
        SetI2cAddr(0x68);
        if (0 != ReadReg((uint8_t) mpu9250_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x71 == val) { return true; }
        SetI2cAddr(0x69);
        if (0 != ReadReg((uint8_t) mpu9250_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x71 == val) { return true; }
        return false;
    }

    int mpu9250_t::Converter(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y,
                             float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int mpu9250_t::Converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x,
                             int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int mpu9250_t::Converter(float *mag_x, float *mag_y, float *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
            return -1;
        }
        if (buf[14 + 7] & MPU9250_AK8963_OVERFLOW) {
            return -1;
        }
        if (mag_x) { *mag_x = magUnit * ak8963Asa[0] * ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = magUnit * ak8963Asa[0] * ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = magUnit * ak8963Asa[0] * ((int16_t) (buf[14 + 8] << 8) | buf[14 + 5]); }
        return 0;
    }

    int mpu9250_t::Converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
            return -1;
        }
        if (buf[14 + 7] & MPU9250_AK8963_OVERFLOW) {
            return -1;
        }
        if (mag_x) { *mag_x = ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = ((int16_t) (buf[14 + 8] << 8) | buf[14 + 5]); }
        return 0;
    }

    int mpu9250_t::Converter(float *temp) {
        if (temp) { *temp = (float) ((int16_t) (buf[6] << 8) | buf[7]) / 333.87f + 21.0f; }
        return 0;
    }

    int mpu9250_t::ReadSensorBlocking() {
        return i2c.ReadBlocking(GetI2cAddr(), (uint8_t) mpu9250_RegMap::ACCEL_XOUT_H, buf, 22);
    }

    int mpu9250_t::ReadSensorNonBlocking() {
        return i2c.ReadNonBlocking(GetI2cAddr(),
                                   (uint8_t) mpu9250_RegMap::ACCEL_XOUT_H,
                                   buf, 22);
    }

    mpu9250_t::mpu9250_t(i2cInterface_t &_i2c) : mpu6500Series_t(_i2c) {
        memset(buf, 0, sizeof(buf));
    }

    int mpu9250_t::SoftReset(void) {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= ReadReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x1);

        //唤起睡眠
        res |= WriteReg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x1);

        return res;
    }

    int imuPtr_t::Load(i2cInterface_t &_i2c) {
        if (icm20602_t(_i2c).Detect()) {
            reset(new icm20602_t(_i2c));
            return 0;
        } else if (mpu6050_t(_i2c).Detect()) {
            reset(new mpu6050_t(_i2c));
            return 0;
        } else if (mpu9250_t(_i2c).Detect()) {
            reset(new mpu9250_t(_i2c));
            return 0;
        }
        return -1;
    }
}