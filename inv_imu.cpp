﻿//
// Created by 17616 on 2020/10/14.
//
#include <cstring>
#include <cmath>
#include"inv_imu.h"

namespace inv {
    int Parser(i2c_interface &_i2c, std::shared_ptr<imu> &ptr) {
        if (icm20602(_i2c).detect()) {
            ptr.reset(new icm20602(_i2c));
            return 0;
        } else if (mpu6050(_i2c).detect()) {
            ptr.reset(new mpu6050(_i2c));
            return 0;
        } else if (mpu9250(_i2c).detect()) {
            ptr.reset(new mpu9250(_i2c));
            return 0;
        }
        return -1;
    }


    int icm20602::init(config _cfg) {
        cfg = _cfg;
        int res = 0;
        if (!detect()) { return -1; }
        //软复位
        res |= soft_reset();

        //打开所有传感器
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_2, 0);

        //1khz采样率
        res |= write_reg((uint8_t) icm20602_RegMap::SMPLRT_DIV, 0);

        //配置陀螺仪lpf
        switch (cfg.gyro_bw) {
            case config::MPU_GBW_250:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 0);
                break;
            case config::MPU_GBW_176:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 1);
                break;
            case config::MPU_GBW_92:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 2);
                break;
            case config::MPU_GBW_41:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 3);
                break;
            case config::MPU_GBW_20:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 4);
                break;
            case config::MPU_GBW_10:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 5);
                break;
            case config::MPU_GBW_5:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 6);
                break;
            default:
                res |= write_reg((uint8_t) icm20602_RegMap::CONFIG, 0);
                break;
        }

        //配置陀螺仪量程
        switch (cfg.gyro_fs) {
            case config::MPU_FS_250dps:
                res |= write_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 0 << 3);
                gyro_unit = 250.0 / 32768;
                break;
            case config::MPU_FS_500dps:
                res |= write_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 1 << 3);
                gyro_unit = 500.0 / 32768;
                break;
            case config::MPU_FS_1000dps:
                res |= write_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 2 << 3);
                gyro_unit = 1000.0 / 32768;
                break;
            case config::MPU_FS_2000dps:
            default:
                res |= write_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, 3 << 3);
                gyro_unit = 2000.0 / 32768;
                break;
        }

        //配置加速度计量程
        switch (cfg.accel_fs) {
            case config::MPU_FS_2G:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 0 << 3);
                accel_unit = 2.0 * 9.8 / 32768;
                break;
            case config::MPU_FS_4G:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 1 << 3);
                accel_unit = 4.0 * 9.8 / 32768;
                break;
            case config::MPU_FS_8G:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 2 << 3);
                accel_unit = 8.0 * 9.8 / 32768;
                break;
            case config::MPU_FS_16G:
            default:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, 3 << 3);
                accel_unit = 16.0 * 9.8 / 32768;
                break;
        }

        //配置加速度计lpf
        switch (cfg.accel_bw) {
            case config::MPU_ABW_218:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 1);
                break;
            case config::MPU_ABW_99:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 2);
                break;
            case config::MPU_ABW_45:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 3);
                break;
            case config::MPU_ABW_21:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 4);
                break;
            case config::MPU_ABW_10:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 5);
                break;
            case config::MPU_ABW_5:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 6);
                break;
            case config::MPU_ABW_420:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 7);
                break;
            default:
                res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG2, 1);
                break;
        }

        //enables interrupt generation by DATA_RDY
        res |= write_reg((uint8_t) icm20602_RegMap::INT_ENABLE, 0x01);
        if (res == 0) {
            isOpen = true;
        }
        return res;
    }

    bool icm20602::detect() {
        uint8_t val;
        addr = 0x68;
        if (0 != read_reg((uint8_t) icm20602_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x12 == val) { return true; }
        addr = 0x69;
        if (0 != read_reg((uint8_t) icm20602_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x12 == val) { return true; }
        return false;
    }

    int icm20602::converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x,
                            int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int icm20602::converter(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y,
                            float *gyro_z) {
        if (acc_x) { *acc_x = accel_unit * ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = accel_unit * ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = accel_unit * ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = gyro_unit * ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = gyro_unit * ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = gyro_unit * ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int icm20602::converter(float *temp) {
        if (temp) { *temp = ((uint16_t) (buf[6] << 8) | buf[7]) / 326.8 + 25.0f; }
        return 0;
    }

    int icm20602::read_sensor_blocking() {
        return i2c.Read(addr, (uint8_t) icm20602_RegMap::ACCEL_XOUT_H, buf, 14);
    }

    int icm20602::read_sensor_NonBlocking() {
        return i2c.ReadNonBlocking(addr,
                                   (uint8_t) icm20602_RegMap::ACCEL_XOUT_H,
                                   buf, 14);
    }

    std::string icm20602::report() {
        std::string rtv;
        rtv += "model:icm20602\t";
        rtv += "addr:";
        rtv += std::to_string((int) addr);
        rtv += '\t';
        return rtv;
    }

    int icm20602::soft_reset(void) {
        if (!detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= read_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x41);

        //唤起睡眠
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x1);

        return res;
    }

    bool icm20602::data_rdy() {
        uint8_t val = 0;
        read_reg((uint8_t) icm20602_RegMap::INT_STATUS, &val);
        return (val & 0x01) == 0x01;
    }

    int icm20602::self_test() {
        if (!is_open()) { return -1; }
        int res = 0;
        config backup_cfg = cfg;
        config st_cfg;
        st_cfg.gyro_fs = config::MPU_FS_250dps;
        st_cfg.accel_fs = config::MPU_FS_2G;
        st_cfg.accel_bw = config::MPU_ABW_99;
        st_cfg.gyro_bw = config::MPU_GBW_92;
        if (0 != init(st_cfg)) {
            init(backup_cfg);
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
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        res |= read_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, &val);
        res |= write_reg((uint8_t) icm20602_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= read_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, &val);
        res |= write_reg((uint8_t) icm20602_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_st[i] += gbuf[i];
                accel_bias_st[i] += abuf[i];
            }
        }

        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] *= 5;   //(32768/2000)*1000 LSB/mg
            accel_bias_regular[i] *= 5;
            gyro_bias_st[i] *= 5;         //(32768/250)*1000 LSB/dps
            accel_bias_st[i] *= 5;
        }


        //计算加速度计自检结果
        uint8_t regs[3];
        int otp_value_zero = 0;
        int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
        int result;

        res |= i2c.Read(addr, (uint8_t) icm20602_RegMap::SELF_TEST_X_ACCEL, regs, 3);
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
        res |= i2c.Read(addr, (uint8_t) icm20602_RegMap::SELF_TEST_X_GYRO, regs, 3);
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
        res |= init(backup_cfg);
        return res | (gyro_result << 1) | accel_result;
    }

    int icm20602::set_bias() {
        int res = 0;
        if (!is_open()) { return -1; }
        int32_t gyro_bias_regular[3];
        int16_t gbuf[3];
        uint8_t val;
        memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
        memset(gbuf, 0, sizeof(gbuf));
        res |= i2c.Write(addr, (uint8_t) icm20602_RegMap::XG_OFFS_USRH,
                         (uint8_t *) gbuf, sizeof(gbuf));
        if (res != 0) { return res; }
        int times;
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 256;
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(NULL, NULL, NULL, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
            }
        }
        if (res != 0) { return res; }
        for (int i = 0; i < 3; ++i) {
            gbuf[i] = -gyro_bias_regular[i] / 256;
        }
        res |= i2c.Write(addr, (uint8_t) icm20602_RegMap::XG_OFFS_USRH,
                         (uint8_t *) gbuf, sizeof(gbuf));
        return res;
    }

    icm20602::icm20602(i2c_interface &_i2c) : imu(_i2c) {
        memset(buf, 0, sizeof(buf));
    }

    int icm20602::converter(float *mag_x, float *mag_y, float *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    int icm20602::converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    bool mpu6050::detect() {
        uint8_t val;
        addr = 0x68;
        if (0 != read_reg((uint8_t) mpu6050_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x68 == val) {
            return true;
        }
        addr = 0x69;
        if (0 != read_reg((uint8_t) mpu6050_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x68 == val) {
            return true;
        }
        return false;
    }

    int mpu6050::self_test() {
        if (!is_open()) { return -1; }
        int res = 0;
        config backup_cfg = cfg;
        config st_cfg;
        st_cfg.gyro_fs = config::MPU_FS_250dps;
        st_cfg.accel_fs = config::MPU_FS_8G;
        if (0 != init(st_cfg)) {
            init(backup_cfg);
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
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        res |= read_reg((uint8_t) mpu6050_RegMap::GYRO_CONFIG, &val);
        res |= write_reg((uint8_t) mpu6050_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= read_reg((uint8_t) mpu6050_RegMap::ACCEL_CONFIG, &val);
        res |= write_reg((uint8_t) mpu6050_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_st[i] += gbuf[i];
                accel_bias_st[i] += abuf[i];
            }
        }

        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] *= 5;   //(32768/2000)*1000 LSB/mg
            accel_bias_regular[i] *= 5;
            gyro_bias_st[i] *= 5;         //(32768/250)*1000 LSB/dps
            accel_bias_st[i] *= 5;
        }

        //开始计算自检结果
        uint8_t regs[4];
        res |= i2c.Read(addr, (uint8_t) mpu6050_RegMap::SELF_TEST_X, regs, 4);
        int a_st[3];
        int g_st[3];
        int ft_a[3];
        int ft_g[3];
        a_st[0] = ((0b111 & (regs[0] >> 5)) << 2) | (0b11 & (regs[3] >> 4));
        a_st[1] = ((0b111 & (regs[1] >> 5)) << 2) | (0b11 & (regs[3] >> 2));
        a_st[2] = ((0b111 & (regs[2] >> 5)) << 2) | (0b11 & (regs[3] >> 0));
        g_st[0] = 0b11111 & regs[0];
        g_st[1] = 0b11111 & regs[1];
        g_st[2] = 0b11111 & regs[2];

        ft_a[0] = (a_st[0] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (a_st[0] - 1) / (1 << 5 - 2));
        ft_a[1] = (a_st[1] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (a_st[1] - 1) / (1 << 5 - 2));
        ft_a[2] = (a_st[2] == 0) ? 0 : 1000 * 4096 * 0.34 * pow(0.92 / 0.34, (a_st[2] - 1) / (1 << 5 - 2));
        ft_g[0] = (g_st[0] == 0) ? 0 : 1000 * 25 * 131 * pow(1.046, g_st[0] - 1);
        ft_g[1] = (g_st[1] == 0) ? 0 : 1000 * -25 * 131 * pow(1.046, g_st[1] - 1);
        ft_g[2] = (g_st[2] == 0) ? 0 : 1000 * 25 * 131 * pow(1.046, g_st[2] - 1);

        for (int i = 0; i < 3; ++i) {
            int str = accel_bias_st[i] - accel_bias_regular[i];
            if (abs(1000 * (str - ft_a[i]) / ft_a[i]) > 140) {
                accel_result = 1;
            }
        }

        for (int i = 0; i < 3; ++i) {
            int str = gyro_bias_st[i] - gyro_bias_regular[i];
            if (abs(1000 * (str - ft_g[i]) / ft_g[i]) > 140) {
                gyro_result = 1;
            }
        }

        //恢复原来的配置
        res |= init(backup_cfg);
        return (gyro_result << 1) | accel_result | res;
    }

    int mpu6050::init(config _cfg) {
        return icm20602::init(_cfg);
    }

    int mpu6050::converter(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) {
        return icm20602::converter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    }

    int mpu6050::converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y,
                           int16_t *gyro_z) {
        return icm20602::converter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    }

    int mpu6050::converter(float *mag_x, float *mag_y, float *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    int mpu6050::converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        (void) mag_x, (void) mag_y, (void) mag_z;
        return 0;
    }

    int mpu6050::converter(float *temp) {
        if (temp) {
            *temp = ((uint16_t) (icm20602::buf[6] << 8)
                     | icm20602::buf[7] - 521) / 340.0 + 35;
        }
        return 0;
    }

    int mpu6050::read_sensor_blocking() {
        return icm20602::read_sensor_blocking();
    }

    int mpu6050::read_sensor_NonBlocking() {
        return icm20602::read_sensor_NonBlocking();
    }

    std::string mpu6050::report() {
        std::string rtv;
        rtv += "model:mpu6050\t";
        rtv += "addr:";
        rtv += std::to_string((int) addr);
        rtv += '\t';
        return rtv;
    }

    int mpu6050::set_bias() { return 0; }

    int mpu6050::soft_reset(void) {
        if (!detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= read_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x40);

        //唤起睡眠
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x0);

        return res;
    }


    int mpu9250::init(config _cfg) {
        int res = icm20602::init((_cfg));
        if (res != 0) { return res; }
        isOpen = false;

        uint8_t val;
        //设置9250内部i2c
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_MST_CTRL, 1 << 4 | 9);//500khz，连续读模式
        res |= write_reg((uint8_t) mpu9250_RegMap::USER_CTRL, 1 << 4 | 9);//开启i2c主模式

        //开始设置ak8963
        //读取id
        res |= sub_i2c_read(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::WIA, &ak8963_DeviceID, 1);
        if (res != 0) { return res; }
        res |= sub_i2c_read(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::INFO, &ak8963_Information, 1);

        //复位并且校准磁力计
        val = MPU9250_AK8963_CNTL2_SRST;
        res |= sub_i2c_write(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL2, &val, 1);
        val = MPU9250_AK8963_POWER_DOWN;
        res |= sub_i2c_write(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);
        val = MPU9250_AK8963_FUSE_ROM_ACCESS;
        res |= sub_i2c_write(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);
        //AK8963 get calibration data
        uint8_t response[3] = {0, 0, 0};
        res |= sub_i2c_read(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::ASAX, response, 3);
        //AK8963_SENSITIVITY_SCALE_FACTOR
        //AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
        //AK8963_ASA[i++] = (s16)((data - 128.0f) *0.00390625f + 1.0f) ;
        AK8963_ASA[0] = (1.0f + 0.00390625f * ((int16_t) (response[0]) - 128));
        AK8963_ASA[1] = (1.0f + 0.00390625f * ((int16_t) (response[1]) - 128));
        AK8963_ASA[2] = (1.0f + 0.00390625f * ((int16_t) (response[2]) - 128));

        val = MPU9250_AK8963_POWER_DOWN;
        res |= sub_i2c_write(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);

        //设置连续读ak8963到fifo
        val = 0x5D;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_MST_CTRL, val);

        val = MPU9250_AK8963_I2C_ADDR | 0x80;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV0_ADDR, val);

        val = (uint8_t) (uint8_t) ak8963_RegMap::ST1;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV0_REG, val);

        val = 0x88;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV0_CTRL, val);

        val = MPU9250_AK8963_CONTINUOUS_MEASUREMENT;
        res |= sub_i2c_write(MPU9250_AK8963_I2C_ADDR, (uint8_t) ak8963_RegMap::CNTL, &val, 1);

        val = 0x09;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, val);

        val = 0x81;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_MST_DELAY_CTRL, val);

        if (res == 0) {
            isOpen = true;
            return 0;
        } else {
            return res;
        }
    }

    std::string mpu9250::report() {
        std::string rtv;
        rtv += "model:mpu9250\t";
        rtv += "addr:";
        rtv += std::to_string((int) addr);
        rtv += '\t';

        rtv += "magnetometer:ak8963\t";
        rtv += "ID:";
        rtv += std::to_string((int) ak8963_DeviceID);
        rtv += '\t';
        rtv += "INF:";
        rtv += std::to_string((int) ak8963_Information);
        rtv += '\t';
        return rtv;
    }

    int mpu9250::sub_i2c_read(unsigned char addr, unsigned char reg, unsigned char *val,
                              unsigned int len) {
        uint8_t index = 0;
        uint8_t status = 0;
        uint32_t timeout = 0;
        uint8_t tmp = 0;
        int res = 0;
        tmp = addr | 0x80;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    //                    inv_icm20602_message(inv_icm20602_warning, "%s:MPU9250-AK8963 Read error: timeout", s->elder.name);
                    return -2;
                }
                res |= read_reg((uint8_t) mpu9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            res |= read_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_DI, val + index);
            index++;
        }
        return res;
    }

    int mpu9250::sub_i2c_write(unsigned char addr, unsigned char reg, const unsigned char *val,
                               unsigned int len) {
        uint32_t timeout = 0;
        uint8_t status = 0;
        uint8_t tmp = 0;
        uint8_t index = 0;
        int res = 0;
        tmp = addr;
        res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = val[index];
            res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_DO, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= write_reg((uint8_t) mpu9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    //                    inv_icm20602_message(inv_icm20602_warning, "%s:MPU9250-AK8963 write error: timeout", s->elder.name);
                    return -2;
                }
                res |= read_reg((uint8_t) mpu9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            if (status & MPU9250_I2C_SLV4_NACK) {
                //                inv_icm20602_message(inv_icm20602_warning, "%s:MPU9250-AK8963 write error: NoACK", s->elder.name);
                return -3;
            }
            index++;
        }
        return res;
    }

    bool mpu9250::detect() {
        uint8_t val;
        addr = 0x68;
        if (0 != read_reg((uint8_t) mpu9250_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x71 == val) { return true; }
        addr = 0x69;
        if (0 != read_reg((uint8_t) mpu9250_RegMap::WHO_AM_I, &val)) { return false; };
        if (0x71 == val) { return true; }
        return false;
    }

    int mpu9250::self_test() {
        if (!is_open()) { return -1; }
        int res = 0;
        config backup_cfg = cfg;
        config st_cfg;
        st_cfg.gyro_fs = config::MPU_FS_250dps;
        st_cfg.accel_fs = config::MPU_FS_2G;
        st_cfg.accel_bw = config::MPU_ABW_99;
        st_cfg.gyro_bw = config::MPU_GBW_92;
        if (0 != init(st_cfg)) {
            init(backup_cfg);
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
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 200;
        res |= read_reg((uint8_t) mpu9250_RegMap::GYRO_CONFIG, &val);
        res |= write_reg((uint8_t) mpu9250_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= read_reg((uint8_t) mpu9250_RegMap::ACCEL_CONFIG, &val);
        res |= write_reg((uint8_t) mpu9250_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_st[i] += gbuf[i];
                accel_bias_st[i] += abuf[i];
            }
        }

        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] *= 5;   //(32768/2000)*1000 LSB/mg
            accel_bias_regular[i] *= 5;
            gyro_bias_st[i] *= 5;         //(32768/250)*1000 LSB/dps
            accel_bias_st[i] *= 5;
        }


        //计算加速度计自检结果
        uint8_t regs[3];
        int otp_value_zero = 0;
        int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
        int result;

        res |= i2c.Read(addr, (uint8_t) mpu9250_RegMap::SELF_TEST_X_ACCEL, regs, 3);
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
                st_shift_ratio[i] = abs(
                        st_shift_cust[i] / st_shift_prod[i] - icm20602::DEF_ST_PRECISION);
                if (st_shift_ratio[i] > icm20602::DEF_ACCEL_ST_SHIFT_DELTA) {
                    //加速度计自检未通过
                    accel_result = 1;
                }
            }
        } else {
            /* Self Test Pass/Fail Criteria B */
            for (i = 0; i < 3; i++) {
                st_shift_cust[i] = abs(accel_bias_st[i] - accel_bias_regular[i]);
                if (st_shift_cust[i] < icm20602::DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000
                    || st_shift_cust[i] > icm20602::DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000) {
                    //加速度计自检未通过
                    accel_result = 1;
                }
            }
        }

        //计算陀螺仪自检结果
        res |= i2c.Read(addr, (uint8_t) mpu9250_RegMap::SELF_TEST_X_GYRO, regs, 3);
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
                if (st_shift_cust[i] < icm20602::DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]) {
                    //陀螺仪自检没过
                    gyro_result = 1;
                }
            } else {
                /* Self Test Pass/Fail Criteria B */
                if (st_shift_cust[i] < icm20602::DEF_GYRO_ST_AL * (32768 / 250) *
                                       icm20602::DEF_ST_PRECISION) {
                    //陀螺仪自检没过
                    gyro_result = 1;
                }
            }
        }

        if (gyro_result == 0) {
            /* Self Test Pass/Fail Criteria C */
            for (i = 0; i < 3; i++) {
                if (abs(gyro_bias_regular[i]) > icm20602::DEF_GYRO_OFFSET_MAX * (32768 / 250) *
                                                icm20602::DEF_ST_PRECISION)
                    //陀螺仪自检没过
                {
                    gyro_result = 1;
                }
            }
        }

        //恢复原来的配置
        res |= init(backup_cfg);
        return (gyro_result << 1) | accel_result | res;
    }

    int mpu9250::set_bias() {
        int res = 0;
        if (!is_open()) { return -1; }
        int32_t gyro_bias_regular[3];
        int16_t gbuf[3];
        uint8_t val;
        memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
        memset(gbuf, 0, sizeof(gbuf));
        res |= i2c.Write(addr, (uint8_t) mpu9250_RegMap::XG_OFFSET_H,
                         (uint8_t *) gbuf, sizeof(gbuf));
        int times;
        times = 100;
        while (times--) { while (!data_rdy()) {}}//丢弃前100个数据
        times = 256;
        while (times--) {
            while (!data_rdy()) {}
            res |= read_sensor_blocking();
            converter(NULL, NULL, NULL, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
            }
        }
        int fs_sel = 0;
        switch (cfg.gyro_fs) {
            case config::MPU_FS_2000dps:
                fs_sel = 3;
                break;
            case config::MPU_FS_1000dps:
                fs_sel = 2;
                break;
            case config::MPU_FS_500dps:
                fs_sel = 1;
                break;
            case config::MPU_FS_250dps:
            default:
                fs_sel = 0;
                break;
        }

        for (int i = 0; i < 3; ++i) {
            gbuf[i] = (gyro_bias_regular[i] / 256) >> 2;
        }
        res |= i2c.Write(addr, (uint8_t) mpu9250_RegMap::XG_OFFSET_H,
                         (uint8_t *) gbuf, sizeof(gbuf));
        return res;
    }

    int mpu9250::converter(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y,
                           float *gyro_z) {
        if (acc_x) { *acc_x = accel_unit * ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = accel_unit * ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = accel_unit * ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = gyro_unit * ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = gyro_unit * ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = gyro_unit * ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int mpu9250::converter(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x,
                           int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int mpu9250::converter(float *mag_x, float *mag_y, float *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
            return -1;
        }
        if (buf[14 + 7] & MPU9250_AK8963_OVERFLOW) {
            return -1;
        }
        if (mag_x) { *mag_x = mag_unit * AK8963_ASA[0] * ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = mag_unit * AK8963_ASA[0] * ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = mag_unit * AK8963_ASA[0] * ((int16_t) (buf[14 + 8] << 8) | buf[14 + 5]); }
        return 0;
    }

    int mpu9250::converter(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
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

    int mpu9250::converter(float *temp) {
        if (temp) { *temp = ((uint16_t) (buf[6] << 8) | buf[7]) / 333.87 + 21.0f; }
        return 0;
    }

    int mpu9250::read_sensor_blocking() {
        return i2c.Read(addr, (uint8_t) mpu9250_RegMap::ACCEL_XOUT_H, buf, 22);
    }

    int mpu9250::read_sensor_NonBlocking() {
        return i2c.ReadNonBlocking(addr,
                                   (uint8_t) mpu9250_RegMap::ACCEL_XOUT_H,
                                   buf, 22);
    }

    mpu9250::mpu9250(i2c_interface &_i2c) : icm20602(_i2c) {
        memset(buf, 0, sizeof(buf));
    }

    int mpu9250::soft_reset(void) {
        if (!detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            res |= read_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, &val);
        } while (val != 0x1);

        //唤起睡眠
        res |= write_reg((uint8_t) icm20602_RegMap::PWR_MGMT_1, 0x1);
    }
}