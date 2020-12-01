#include <cstring>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cfloat>
#include <initializer_list>
#include"inv_imu.hpp"
namespace inv {
    template<typename Val = int>
    class WeakMap {
    private:
        std::vector<float> key;
        std::vector<Val> val;
    public:
        WeakMap() = default;
        WeakMap(const std::initializer_list<std::pair<float, Val> > &list) : WeakMap() {
            for (auto i:list) {
                key.push_back(i.first);
                val.push_back(i.second);
            }
        }
        void Insert(float _key, Val _val) {
            key.push_back(_key);
            val.push_back(_val);
        }
        const Val &operator[](float _key) const {
            int n = 0;
            float distance = FLT_MAX;
            float buf;
            for (int i = 0; i < key.size(); ++i) {
                buf = key[i] - _key;
                buf *= buf;
                if (buf < distance) {
                    distance = buf;
                    n = i;
                }
            }
            return val[n];
        }
    };

    const WeakMap<int> MPU_ACCEL_FS_MAP({
                                                {Config::MPU_FS_2G,  0},
                                                {Config::MPU_FS_4G,  1},
                                                {Config::MPU_FS_8G,  2},
                                                {Config::MPU_FS_16G, 3}
                                        });
    const WeakMap<int> MPU_GYRO_FS_MAP({
                                               {Config::MPU_FS_250dps,  0},
                                               {Config::MPU_FS_500dps,  1},
                                               {Config::MPU_FS_1000dps, 2},
                                               {Config::MPU_FS_2000dps, 3}
                                       });
    const WeakMap<float> MPU_ACCEL_UNIT_IN_G({
                                                     {Config::MPU_FS_2G,  2.0 / 32768.0},
                                                     {Config::MPU_FS_4G,  4.0 / 32768.0},
                                                     {Config::MPU_FS_8G,  8.0 / 32768.0},
                                                     {Config::MPU_FS_16G, 16.0 / 32768.0}
                                             });
    const WeakMap<float> MPU_GYRO_UNIT_IN_DPS({
                                                      {Config::MPU_FS_250dps,  250.0 / 32768.0},
                                                      {Config::MPU_FS_500dps,  500.0 / 32768.0},
                                                      {Config::MPU_FS_1000dps, 1000.0 / 32768.0},
                                                      {Config::MPU_FS_2000dps, 2000.0 / 32768.0}
                                              });
    const WeakMap<float> MPU_ACCEL_UNIT_FROM_G({
                                                       {Config::MPU_UNIT_MetersPerSquareSecond, 9.8},
                                                       {Config::MPU_UNIT_G,                     1.0},
                                                       {Config::MPU_UNIT_mG,                    1000.0}
                                               });
    const WeakMap<float> MPU_GYRO_UNIT_FROM_DPS({
                                                        {Config::MPU_UNIT_DegPerSec,            1.0},
                                                        {Config::MPU_UNIT_RadPerSec, M_PI / 180.0},
                                                        {Config::MPU_UNIT_RevolutionsPerMinute, 60.0 / 360.0},
                                                });
    const WeakMap<int> MPU9250_GBW_MAP({
                                               {250, 0},
                                               {184, 1},
                                               {92,  2},
                                               {41,  3},
                                               {20,  4},
                                               {10,  5},
                                               {5,   6}
                                       });
    const WeakMap<int> MPU9250_ABW_MAP({
                                               {218.1, 1},
                                               {99,    2},
                                               {44.8,  3},
                                               {21.2,  4},
                                               {10.2,  5},
                                               {5.05,  6},
                                               {420,   7}
                                       });
    const WeakMap<int> &MPU6050_GBW_MAP = MPU9250_GBW_MAP;
    const WeakMap<int> &ICM20602_GBW_MAP = MPU9250_GBW_MAP;
    const WeakMap<int> &ICM20602_ABW_MAP = MPU9250_ABW_MAP;
    const WeakMap<int> ICM20948_GBW_MAP({
                                                {196.6, 0},
                                                {151.8, 1},
                                                {119.5, 2},
                                                {51.2,  3},
                                                {23.9,  4},
                                                {11.6,  5},
                                                {5.7,   6},
                                                {361.4, 7}
                                        });
    const WeakMap<int> &ICM20948_ABW_MAP = MPU9250_ABW_MAP;

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

    constexpr static const unsigned int MPU9250_I2C_SLV4_EN = 0x80;
    constexpr static const unsigned int MPU9250_I2C_SLV4_DONE = 0x40;
    constexpr static const unsigned int MPU9250_I2C_SLV4_NACK = 0x10;
    constexpr static const unsigned int MPU9250_AK8963_I2C_ADDR = 0x0C;
    constexpr static const unsigned int ICM20948_AK09916_I2C_ADDR = 0x0C;
    constexpr static const unsigned int MPU9250_AK8963_POWER_DOWN = 0x10;
    constexpr static const unsigned int MPU9250_AK8963_FUSE_ROM_ACCESS = 0x1F;
//    constexpr static const unsigned int MPU9250_AK8963_SINGLE_MEASUREMENT = 0x11;
    constexpr static const unsigned int MPU9250_AK8963_CONTINUOUS_MEASUREMENT = 0x16; //MODE 2
    constexpr static const unsigned int MPU9250_AK8963_DATA_READY = (0x01);
    constexpr static const unsigned int MPU9250_AK8963_DATA_OVERRUN = (0x02);
    //constexpr static const unsigned int MPU9250_AK8963_OVERFLOW = (0x80);
    constexpr static const unsigned int MPU9250_AK8963_OVERFLOW = (0x08);
//    constexpr static const unsigned int MPU9250_AK8963_DATA_ERROR = (0x40);
    constexpr static const unsigned int MPU9250_AK8963_CNTL2_SRST = 0x01;



    std::string ICM20602::Report() {
        std::string rtv;
        rtv += "model:icm20602\t";
        rtv += "addr:";
        rtv += std::to_string((int) i2cTransfer.slaveAddress);
        rtv += '\t';
        return rtv;
    }

    int ICM20602::Convert(float *temp) {
        if (temp) { *temp = (float) ((int16_t) (buf[6] << 8) | buf[7]) / 326.8f + 25.0f; }
        return 0;
    }

    int ICM20602::SoftReset(void) {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
            INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x41", val);
        } while (val != 0x41);

        //唤起睡眠
        ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) ICM20602_RegMap::PWR_MGMT_1, &val);
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::PWR_MGMT_1, 0x1);

        return res;
    }

    int ICM20602::Init(Config _cfg) {
        cfg = _cfg;
        isOpen = false;
        int res = 0;
        if (!Detect()) { return -1; }
        //软复位
        res |= SoftReset();

        //打开所有传感器
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::PWR_MGMT_2, 0);

        //1khz采样率
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::SMPLRT_DIV, 0);

        //配置陀螺仪lpf
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::CONFIG, ICM20948_GBW_MAP[cfg.gyroBandwidth]);

        //配置陀螺仪量程和单位
        gyroUnit = MPU_GYRO_UNIT_IN_DPS[cfg.gyroFullScale] * MPU_GYRO_UNIT_FROM_DPS[cfg.gyroUnit];
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::GYRO_CONFIG, MPU_GYRO_FS_MAP[cfg.gyroFullScale] << 3u);

        //配置加速度计量程和单位
        accelUnit = MPU_ACCEL_UNIT_IN_G[cfg.accelFullScale] * MPU_ACCEL_UNIT_FROM_G[cfg.accelUnit];
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::ACCEL_CONFIG, MPU_ACCEL_FS_MAP[cfg.accelFullScale] << 3u);

        //配置加速度计lpf
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::ACCEL_CONFIG2, ICM20602_ABW_MAP[cfg.accelBandwidth]);

        //开启数据更新中断
        res |= EnableDataReadyInt();

        if (res == 0) {
            isOpen = true;
        }
        return res;
    }

    bool ICM20602::Detect() {
        uint8_t val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x68; }
        ReadReg((uint8_t) ICM20602_RegMap::WHO_AM_I, &val);
        if (0x12 == val) {
            return true;
        }
        val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x69; }
        ReadReg((uint8_t) ICM20602_RegMap::WHO_AM_I, &val);
        if (0x12 == val) {
            return true;
        }
        return false;
    }

    int ICM20602::SelfTest() {
        if (!IsOpen()) { return -1; }
        int res = 0;
        Config backup_cfg = cfg;
        Config st_cfg;
        st_cfg.gyroFullScale = Config::MPU_FS_250dps;
        st_cfg.accelFullScale = Config::MPU_FS_2G;
        st_cfg.accelBandwidth = Config::MPU_ABW_99;
        st_cfg.gyroBandwidth = Config::MPU_GBW_92;
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
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        res |= ReadReg((uint8_t) ICM20602_RegMap::GYRO_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= ReadReg((uint8_t) ICM20602_RegMap::ACCEL_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) ICM20602_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        times = 20;
        while (times--) { while (!DataReady()) {}}//丢弃前20个数据
        times = 20;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
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

        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_X_ACCEL, regs);
        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_Y_ACCEL, regs + 1);
        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_Z_ACCEL, regs + 2);
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
                    INV_DEBUG("accel[%d] st fail,result = %d,it demands less than %d", i, st_shift_ratio[i],
                              DEF_ACCEL_ST_SHIFT_DELTA);
                } else {
                    INV_TRACE("accel[%d] st result = %d,it demands less than %d", i, st_shift_ratio[i],
                              DEF_ACCEL_ST_SHIFT_DELTA);
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
                    INV_DEBUG("accel[%d] st fail,result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                              DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
                } else {
                    INV_TRACE("accel[%d] st result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                              DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
                }
            }
        }

        //计算陀螺仪自检结果
        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_X_GYRO, regs);
        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_Y_GYRO, regs + 1);
        res |= ReadReg((uint8_t) ICM20602_RegMap::SELF_TEST_Z_GYRO, regs + 2);
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
                    INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
                }
            } else {
                /* Self Test Pass/Fail Criteria B */
                if (st_shift_cust[i] < DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION) {
                    //陀螺仪自检没过
                    gyro_result = 1;
                    INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
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
                    INV_DEBUG("gyro[%d] st fail,result = %d,ift demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                              DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                              DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
                }
            }
        }

        //恢复原来的配置
        res |= Init(backup_cfg);
        return res | (gyro_result << 1) | accel_result;
    }

    bool ICM20602::DataReady() {
        uint8_t val = 0;
        ReadReg((uint8_t) ICM20602_RegMap::INT_STATUS, &val);
        return (val & 0x01) == 0x01;
    }

    int ICM20602::EnableDataReadyInt() {
        return ModifyReg((uint8_t) ICM20602_RegMap::INT_ENABLE, 0x01, 0x01);
    }

    int ICM20602::ReadSensorBlocking() {
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) ICM20602_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 14;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 15;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int ICM20602::ReadSensorNonBlocking() {
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) ICM20602_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 14;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferNonBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 15;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferNonBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int ICM20602::Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11])); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) ((buf[12] << 8) | buf[13])); }
        return 0;
    }

    int ICM20602::Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_y) { *gyro_y = ((int16_t) ((buf[10] << 8) | buf[11])); }
        if (gyro_z) { *gyro_z = ((int16_t) ((buf[12] << 8) | buf[13])); }
        return 0;
    }

    int ICM20602::Convert(float *mag_x, float *mag_y, float *mag_z) {
        return 0;
    }

    int ICM20602::Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        return 0;
    }

    int MPU6050::SelfTest() {
        if (!IsOpen()) { return -1; }
        int res = 0;
        Config backup_cfg = cfg;
        Config st_cfg;
        st_cfg.gyroFullScale = Config::MPU_FS_250dps;
        st_cfg.accelFullScale = Config::MPU_FS_8G;
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
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        res |= ReadReg((uint8_t) MPU6050_RegMap::GYRO_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= ReadReg((uint8_t) MPU6050_RegMap::ACCEL_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        times = 20;
        while (times--) { while (!DataReady()) {}}//丢弃前100个数据
        times = 20;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
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

        res |= ReadReg((uint8_t) MPU6050_RegMap::SELF_TEST_X, regs);
        res |= ReadReg((uint8_t) MPU6050_RegMap::SELF_TEST_Y, regs + 1);
        res |= ReadReg((uint8_t) MPU6050_RegMap::SELF_TEST_Z, regs + 2);
        res |= ReadReg((uint8_t) MPU6050_RegMap::SELF_TEST_A, regs + 3);
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

        a_st[0] &= (32 - 1);
        a_st[1] &= (32 - 1);
        a_st[2] &= (32 - 1);
        g_st[0] &= (32 - 1);
        g_st[1] &= (32 - 1);
        g_st[2] &= (32 - 1);

        ft_a[0] = 1000 * accelSelfTestEquation[a_st[0]];
        ft_a[1] = 1000 * accelSelfTestEquation[a_st[1]];
        ft_a[2] = 1000 * accelSelfTestEquation[a_st[2]];
        ft_g[0] = 1000 * gyroSelfTestEquation[g_st[0]];
        ft_g[1] = -1000 * gyroSelfTestEquation[g_st[1]];
        ft_g[2] = 1000 * gyroSelfTestEquation[g_st[2]];

        for (int i = 0; i < 3; ++i) {
            int str = accel_bias_st[i] - accel_bias_regular[i];
            float Change_from_factory_trim = (float) (str - ft_a[i]) / ft_a[i];
            if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
                INV_DEBUG("6050 accel[%d] self test fail,result = %f,it demands >-0.14 && <0.14", i,
                          Change_from_factory_trim);
                accel_result = 1;
            } else {
                INV_TRACE("6050 accel[%d] self test result = %f,it demands >-0.14 && <0.14", i,
                          Change_from_factory_trim);
            }
        }

        for (int i = 0; i < 3; ++i) {
            int str = gyro_bias_st[i] - gyro_bias_regular[i];
            float Change_from_factory_trim = (float) (str - ft_g[i]) / ft_g[i];
            if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
                INV_DEBUG("6050 gryo[%d] self test fail,result = %f,it demands >-0.14 && <0.14", i,
                          Change_from_factory_trim);
                gyro_result = 1;
            } else {
                INV_TRACE("6050 gryo[%d] self test result = %f,it demands >-0.14 && <0.14", i,
                          Change_from_factory_trim);
            }
        }

        //恢复原来的配置
        res |= Init(backup_cfg);
        return (gyro_result << 1) | accel_result | res;
    }


    int MPU6050::Convert(float *temp) {
        if (temp) {
            *temp = (float) ((int16_t) ((buf[6] << 8) | buf[7] - 521)) / 340.0f + 35;
        }
        return 0;
    }

    std::string MPU6050::Report() {
        std::string rtv;
        rtv += "model:mpu6050\t";
        rtv += "addr:";
        rtv += std::to_string((int) i2cTransfer.slaveAddress);
        rtv += '\t';
        return rtv;
    }

    int MPU6050::SoftReset(void) {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
            INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x40", val);
        } while (val != 0x40);

        //唤起睡眠
        ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU6050_RegMap::PWR_MGMT_1, &val);
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::PWR_MGMT_1, 0x0);

        return res;
    }
    int MPU6050::Init(Config _cfg) {
        cfg = _cfg;
        isOpen = false;
        int res = 0;
        if (!Detect()) { return -1; }
        //软复位
        res |= SoftReset();

        //打开所有传感器
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::PWR_MGMT_2, 0);

        //1khz采样率
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::SMPLRT_DIV, 0);

        //配置陀螺仪lpf
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::CONFIG, MPU6050_GBW_MAP[cfg.gyroBandwidth]);

        //配置陀螺仪量程和单位
        gyroUnit = MPU_GYRO_UNIT_IN_DPS[cfg.gyroFullScale] * MPU_GYRO_UNIT_FROM_DPS[cfg.gyroUnit];
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::GYRO_CONFIG, MPU_GYRO_FS_MAP[cfg.gyroFullScale] << 3u);

        //配置加速度计量程和单位
        accelUnit = MPU_ACCEL_UNIT_IN_G[cfg.accelFullScale] * MPU_ACCEL_UNIT_FROM_G[cfg.accelUnit];
        res |= WriteRegVerified((uint8_t) MPU6050_RegMap::ACCEL_CONFIG, MPU_ACCEL_FS_MAP[cfg.accelFullScale] << 3u);

        //开启数据更新中断
        res |= EnableDataReadyInt();

        if (res == 0) {
            isOpen = true;
        }
        return res;
    }
    bool MPU6050::Detect() {
        uint8_t val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x68; };
        ReadReg((uint8_t) MPU6050_RegMap::WHO_AM_I, &val);
        if (0x68 == val) {
            return true;
        }
        val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x69; };
        ReadReg((uint8_t) MPU6050_RegMap::WHO_AM_I, &val);
        if (0x68 == val) {
            return true;
        }
        return false;
    }

    bool MPU6050::DataReady() {
        uint8_t val = 0;
        ReadReg((uint8_t) MPU6050_RegMap::INT_STATUS, &val);
        return (val & 0x01) == 0x01;
    }

    int MPU6050::EnableDataReadyInt() {
        return ModifyReg((uint8_t) MPU6050_RegMap::INT_ENABLE, 0x01, 0x01);
    }

    int MPU6050::ReadSensorBlocking() {
        i2cTransfer.direction = I2C::Transfer::Read;
        i2cTransfer.subAddress = (uint8_t) MPU6050_RegMap::ACCEL_XOUT_H;
        i2cTransfer.data = buf;
        i2cTransfer.dataSize = 14;
        return i2c->masterTransferBlocking(i2cTransfer);
    }

    int MPU6050::ReadSensorNonBlocking() {
        i2cTransfer.direction = I2C::Transfer::Read;
        i2cTransfer.subAddress = (uint8_t) MPU6050_RegMap::ACCEL_XOUT_H;
        i2cTransfer.data = buf;
        i2cTransfer.dataSize = 14;
        return i2c->masterTransferNonBlocking(i2cTransfer);
    }

    int MPU6050::Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        return 0;
    }

    int MPU6050::Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11])); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) ((buf[12] << 8) | buf[13])); }
        return 0;
    }

    int MPU6050::Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_y) { *gyro_y = ((int16_t) ((buf[10] << 8) | buf[11])); }
        if (gyro_z) { *gyro_z = ((int16_t) ((buf[12] << 8) | buf[13])); }
        return 0;
    }

    int MPU6050::Convert(float *mag_x, float *mag_y, float *mag_z) {
        return 0;
    }


    int MPU9250::Init(Config _cfg) {
        cfg = _cfg;
        isOpen = false;
        int res = 0;
        if (!Detect()) { return -1; }
        //软复位
        res |= SoftReset();

        //打开所有传感器
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::PWR_MGMT_2, 0);

        //1khz采样率
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::SMPLRT_DIV, 0);

        //配置陀螺仪lpf
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::CONFIG, MPU9250_GBW_MAP[cfg.gyroBandwidth]);

        //配置陀螺仪量程和单位
        gyroUnit = MPU_GYRO_UNIT_IN_DPS[cfg.gyroFullScale] * MPU_GYRO_UNIT_FROM_DPS[cfg.gyroUnit];
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::GYRO_CONFIG, MPU_GYRO_FS_MAP[cfg.gyroFullScale] << 3u);

        //配置加速度计量程和单位
        accelUnit = MPU_ACCEL_UNIT_IN_G[cfg.accelFullScale] * MPU_ACCEL_UNIT_FROM_G[cfg.accelUnit];
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::ACCEL_CONFIG, MPU_ACCEL_FS_MAP[cfg.accelFullScale] << 3u);

        //配置加速度计lpf
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::ACCEL_CONFIG2, MPU9250_ABW_MAP[cfg.accelBandwidth]);

        //开启数据更新中断
        res |= EnableDataReadyInt();

        if (res != 0) { return res; }
        isOpen = false;

        uint8_t val;
        //设置9250内部i2c
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_MST_CTRL, 1 << 4 | 9);//500khz，连续读模式
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::USER_CTRL, 1 << 5);//开启i2c主模式

        //开始设置ak8963
        //读取id
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::WIA, &ak8963DeviceId, 1);
        if (res != 0) { return res; }
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::INFO, &ak8963Information, 1);

        //复位并且校准磁力计
        val = MPU9250_AK8963_CNTL2_SRST;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::CNTL2, &val, 1);
        val = MPU9250_AK8963_POWER_DOWN;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::CNTL, &val, 1);
        val = MPU9250_AK8963_FUSE_ROM_ACCESS;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::CNTL, &val, 1);
        //AK8963 get calibration data
        uint8_t response[3] = {0, 0, 0};
        res |= SubI2cRead(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::ASAX, response, 3);
        INV_TRACE("0x%x 0x%x 0x%x at AK8963_RegMap::ASAX", response[0], response[1], response[2]);
        //AK8963_SENSITIVITY_SCALE_FACTOR
        //ak8963Asa[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
        //ak8963Asa[i++] = (s16)((data - 128.0f) *0.00390625f + 1.0f) ;
        ak8963Asa[0] = (1.0f + 0.00390625f * ((int16_t) (response[0]) - 128));
        ak8963Asa[1] = (1.0f + 0.00390625f * ((int16_t) (response[1]) - 128));
        ak8963Asa[2] = (1.0f + 0.00390625f * ((int16_t) (response[2]) - 128));

        INV_TRACE("%f %f %f at ak8963Asa", ak8963Asa[0], ak8963Asa[1], ak8963Asa[2]);
        val = MPU9250_AK8963_POWER_DOWN;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::CNTL, &val, 1);

        //设置连续读ak8963到fifo
        val = 0x5D;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_MST_CTRL, val);

        val = MPU9250_AK8963_I2C_ADDR | 0x80;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_SLV0_ADDR, val);

        val = (uint8_t) (uint8_t) AK8963_RegMap::ST1;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_SLV0_REG, val);

        val = 0x88;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_SLV0_CTRL, val);

        val = MPU9250_AK8963_CONTINUOUS_MEASUREMENT;
        res |= SubI2cWrite(MPU9250_AK8963_I2C_ADDR, (uint8_t) AK8963_RegMap::CNTL, &val, 1);

        val = 0x09;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_SLV4_CTRL, val);

        val = 0x81;
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::I2C_MST_DELAY_CTRL, val);

        if (res == 0) {
            isOpen = true;
            return 0;
        } else {
            return res;
        }
    }

    std::string MPU9250::Report() {
        std::string rtv;
        rtv += "model:mpu9250\t";
        rtv += "addr:";
        rtv += std::to_string((int) i2cTransfer.slaveAddress);
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

    int MPU9250::SubI2cRead(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len) {
        uint8_t index = 0;
        uint8_t status = 0;
        uint32_t timeout = 0;
        uint8_t tmp = 0;
        int res = 0;
        tmp = addr | 0x80;
        res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    INV_DEBUG("SubI2cRead time out");
                    return -2;
                }
                res |= ReadReg((uint8_t) MPU9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            res |= ReadReg((uint8_t) MPU9250_RegMap::I2C_SLV4_DI, val + index);
            index++;
        }
        return res;
    }

    int MPU9250::SubI2cWrite(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len) {
        uint32_t timeout = 0;
        uint8_t status = 0;
        uint8_t tmp = 0;
        uint8_t index = 0;
        int res = 0;
        tmp = addr;
        res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_REG, tmp);
            tmp = val[index];
            res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_DO, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint8_t) MPU9250_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    INV_DEBUG("SubI2cWrite time out");
                    return -2;
                }
                res |= ReadReg((uint8_t) MPU9250_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            if (status & MPU9250_I2C_SLV4_NACK) {
                INV_DEBUG("SubI2cWrite no ack");
                return -3;
            }
            index++;
        }
        return res;
    }

    int MPU9250::Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y,
                         float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int MPU9250::Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x,
                         int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) (buf[0] << 8) | buf[1]); }
        if (acc_y) { *acc_y = ((int16_t) (buf[2] << 8) | buf[3]); }
        if (acc_z) { *acc_z = ((int16_t) (buf[4] << 8) | buf[5]); }
        if (gyro_x) { *gyro_x = ((int16_t) (buf[8] << 8) | buf[9]); }
        if (gyro_y) { *gyro_y = ((int16_t) (buf[10] << 8) | buf[11]); }
        if (gyro_z) { *gyro_z = ((int16_t) (buf[12] << 8) | buf[13]); }
        return 0;
    }

    int MPU9250::Convert(float *mag_x, float *mag_y, float *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
//            INV_TRACE("0x%x at buf[14 + 0]", (int) buf[14 + 0]);
            return -1;
        }
        if (buf[14 + 7] & MPU9250_AK8963_OVERFLOW) {
//            INV_TRACE("0x%x at buf[14 + 7]", (int) buf[14 + 7]);
            return -1;
        }
        if (mag_x) { *mag_x = magUnit * ak8963Asa[0] * ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = magUnit * ak8963Asa[1] * ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = magUnit * ak8963Asa[2] * ((int16_t) (buf[14 + 6] << 8) | buf[14 + 5]); }
        return 0;
    }

    int MPU9250::Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
//            INV_TRACE("0x%x at buf[14 + 0]", (int) buf[14 + 0]);
            return -1;
        }
        if (buf[14 + 7] & MPU9250_AK8963_OVERFLOW) {
//            INV_TRACE("0x%x at buf[14 + 7]", (int) buf[14 + 7]);
            return -1;
        }
        if (mag_x) { *mag_x = ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = ((int16_t) (buf[14 + 6] << 8) | buf[14 + 5]); }
        return 0;
    }

    int MPU9250::Convert(float *temp) {
        if (temp) { *temp = (float) ((int16_t) (buf[6] << 8) | buf[7]) / 333.87f + 21.0f; }
        return 0;
    }

    int MPU9250::ReadSensorBlocking() {
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) MPU9250_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 22;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) MPU9250_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 23;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int MPU9250::ReadSensorNonBlocking() {
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) MPU9250_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 22;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferNonBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) MPU9250_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 23;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferNonBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int MPU9250::SoftReset() {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //复位
        res |= WriteReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
            INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x1", val);
        } while (val != 0x1);

        //唤起睡眠
        ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint8_t) MPU9250_RegMap::PWR_MGMT_1, &val);
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::PWR_MGMT_1, 0x1);

        return res;
    }
    bool MPU9250::Detect() {
        uint8_t val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x68; };
        ReadReg((uint8_t) MPU9250_RegMap::WHO_AM_I, &val);
        if (0x71 == val) {
            return true;
        }
        val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x69; };
        ReadReg((uint8_t) MPU9250_RegMap::WHO_AM_I, &val);
        if (0x71 == val) {
            return true;
        }
        return false;
    }
    int MPU9250::SelfTest() {
        if (!IsOpen()) { return -1; }
        int res = 0;
        Config backup_cfg = cfg;
        Config st_cfg;
        st_cfg.gyroFullScale = Config::MPU_FS_250dps;
        st_cfg.accelFullScale = Config::MPU_FS_2G;
        st_cfg.accelBandwidth = Config::MPU_ABW_99;
        st_cfg.gyroBandwidth = Config::MPU_GBW_92;
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
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        res |= ReadReg((uint8_t) MPU9250_RegMap::GYRO_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
        res |= ReadReg((uint8_t) MPU9250_RegMap::ACCEL_CONFIG, &val);
        res |= WriteRegVerified((uint8_t) MPU9250_RegMap::ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
        times = 20;
        while (times--) { while (!DataReady()) {}}//丢弃前20个数据
        times = 20;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
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


        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_X_ACCEL, regs);
        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_Y_ACCEL, regs + 1);
        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_Z_ACCEL, regs + 2);
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
                    INV_DEBUG("accel[%d] st fail,result = %d,it demands less than %d", i, st_shift_ratio[i],
                              DEF_ACCEL_ST_SHIFT_DELTA);
                } else {
                    INV_TRACE("accel[%d] st result = %d,it demands less than %d", i, st_shift_ratio[i],
                              DEF_ACCEL_ST_SHIFT_DELTA);
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
                    INV_DEBUG("accel[%d] st fail,result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                              DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
                } else {
                    INV_TRACE("accel[%d] st result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                              DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
                }
            }
        }

        //计算陀螺仪自检结果
        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_X_GYRO, regs);
        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_Y_GYRO, regs + 1);
        res |= ReadReg((uint8_t) MPU9250_RegMap::SELF_TEST_Z_GYRO, regs + 2);
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
                    INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
                }
            } else {
                /* Self Test Pass/Fail Criteria B */
                if (st_shift_cust[i] < DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION) {
                    //陀螺仪自检没过
                    gyro_result = 1;
                    INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                              DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
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
                    INV_DEBUG("gyro[%d] st fail,result = %d,ift demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                              DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
                } else {
                    INV_TRACE("gyro[%d] st result = %d,it demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                              DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
                }
            }
        }

        //恢复原来的配置
        res |= Init(backup_cfg);
        return res | (gyro_result << 1u) | accel_result;
    }

    bool MPU9250::DataReady() {
        uint8_t val = 0;
        ReadReg((uint8_t) MPU9250_RegMap::INT_STATUS, &val);
        return (val & 0x01u) == 0x01u;
    }

    int MPU9250::EnableDataReadyInt() {
        return ModifyReg((uint8_t) MPU9250_RegMap::INT_ENABLE, 0x01, 0x01);
    }

    int IMU_Ptr::Load(I2C &_i2c, uint16_t _addr) {
        if (ICM20602(_i2c, _addr).Detect()) {
            INV_TRACE("icm20602 detected");
            reset(new ICM20602(_i2c, _addr));
            return 0;
        } else if (MPU6050(_i2c, _addr).Detect()) {
            INV_TRACE("mpu6050 detected");
            reset(new MPU6050(_i2c, _addr));
            return 0;
        } else if (MPU9250(_i2c, _addr).Detect()) {
            INV_TRACE("mpu9250 detected");
            reset(new MPU9250(_i2c, _addr));
            return 0;
        } else if (ICM20600(_i2c, _addr).Detect()) {
            INV_TRACE("icm20600 detected");
            reset(new ICM20600(_i2c, _addr));
            return 0;
        } else if (ICM20948(_i2c, _addr).Detect()) {
            INV_TRACE("icm20948 detected");
            reset(new ICM20948(_i2c, _addr));
            return 0;
        }
        return -1;
    }
    int IMU_Ptr::Load(SPI &_spi) {
        if (ICM20602(_spi).Detect()) {
            INV_TRACE("icm20602 detected");
            reset(new ICM20602(_spi));
            return 0;
        } else if (MPU9250(_spi).Detect()) {
            INV_TRACE("mpu9250 detected");
            reset(new MPU9250(_spi));
            return 0;
        } else if (ICM20600(_spi).Detect()) {
            INV_TRACE("icm20600 detected");
            reset(new ICM20600(_spi));
            return 0;
        } else if (ICM20948(_spi).Detect()) {
            INV_TRACE("icm20948 detected");
            reset(new ICM20948(_spi));
            return 0;
        }
        return -1;
    }

    int IMU::WriteReg(uint8_t reg, uint8_t val) {
        int res = 0;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = reg;
            i2cTransfer.data = &val;
            i2cTransfer.dataSize = 1;
            i2cTransfer.direction = I2C::Transfer::Write;
            res = i2c->masterTransferBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c write return code = %d", res);
            }
        } else {
            uint8_t txb[2];
            uint8_t rxb[2];
            txb[0] = (reg & 0x7fU);
            txb[1] = val;
            spiTransfer.dataSize = 2;
            spiTransfer.rxData = rxb;
            spiTransfer.txData = txb;
            res = spi->masterTransferBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi write return code = %d", res);
            }
        }
        return res;
    }

    int IMU::ReadReg(uint8_t reg, uint8_t *val) {
        int res = 0;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = reg;
            i2cTransfer.data = val;
            i2cTransfer.dataSize = 1;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            uint8_t txb[2];
            uint8_t rxb[2];
            txb[0] = (1U << 7U) | (reg & 0x7f);
            spiTransfer.dataSize = 2;
            spiTransfer.rxData = rxb;
            spiTransfer.txData = txb;
            res = spi->masterTransferBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            } else {
                *val = rxb[1];
            }

        }
        return res;
    }

    int IMU::ModifyReg(uint8_t reg, uint8_t val, uint8_t mask) {
        uint8_t regVal;
        int res = 0;
        res |= ReadReg(reg, &regVal);
        res |= WriteRegVerified(reg, (regVal & (~mask)) | (val & mask));
        res |= ReadReg(reg, &regVal);
        if ((regVal & mask) != (val & mask)) {
            INV_DEBUG("imu rw error");
            res |= -1;
        }
        return res;
    }

    int IMU::WriteRegVerified(uint8_t reg, uint8_t val) {
        uint8_t regVal;
        int res = 0;
        res |= WriteReg(reg, val);
        res |= ReadReg(reg, &regVal);
        if (res == 0 && val != regVal) {
            res |= WriteReg(reg, val);
            res |= ReadReg(reg, &regVal);
            if (res == 0 && val != regVal) {
                INV_DEBUG("imu  rw error");
                res |= -1;
            }
        }
        return res;
    }
    IMU::IMU(I2C &_i2c, uint16_t _addr) : i2c(&_i2c), spi(nullptr), isOpen(false) {
        i2cTransfer.slaveAddress = _addr;
        if (_addr == SlaveAddressAutoDetect) {
            addrAutoDetect = true;
        } else {
            addrAutoDetect = false;
        }
    }
    IMU::IMU(SPI &_spi) : i2c(nullptr), spi(&_spi), addrAutoDetect(false), isOpen(false) {}
    bool IMU::IsOpen() { return isOpen; }
    bool ICM20600::Detect() {
        uint8_t val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x68; };
        ReadReg((uint8_t) ICM20602_RegMap::WHO_AM_I, &val);
        if (0x11 == val) {
            return true;
        }
        val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x69; };
        ReadReg((uint8_t) ICM20602_RegMap::WHO_AM_I, &val);
        if (0x11 == val) {
            return true;
        }
        return false;
    }
    std::string ICM20600::Report() {
        std::string rtv;
        rtv += "model:icm20600\t";
        rtv += "addr:";
        rtv += std::to_string((int) i2cTransfer.slaveAddress);
        rtv += '\t';
        return rtv;
    }

    int ICM20948::SoftReset() {
        if (!Detect()) { return -1; }
        int res = 0;
        uint8_t val;
        //先选择bank0
        SwitchBank(0);
        //复位
        res |= WriteReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, 0x80);
        //等待复位成功
        do {
            ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
            INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x41", val);
        } while (val != 0x41);

        //唤起睡眠
        ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
        ReadReg((uint16_t) ICM20948_RegMap::PWR_MGMT_1, &val);
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::PWR_MGMT_1, 0x1);
        return res;
    }
    bool ICM20948::Detect() {
        uint8_t val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x68; };
        SwitchBank(0);
        ReadReg((uint16_t) ICM20948_RegMap::WHO_AM_I, &val);
        if (0xEA == val) {
            return true;
        }
        val = 0;
        if (addrAutoDetect) { i2cTransfer.slaveAddress = 0x69; };
        SwitchBank(0);
        ReadReg((uint16_t) ICM20948_RegMap::WHO_AM_I, &val);
        if (0xEA == val) {
            return true;
        }
        return false;
    }

    int ICM20948::SwitchBank(int _bank) {
        bank = _bank;
        return IMU::WriteRegVerified((uint8_t) ICM20948_RegMap::REG_BANK_SEL, _bank << 4);
    }
    int ICM20948::WriteReg(uint16_t reg, const uint8_t val) {
        if (bank != reg >> 8) {
            SwitchBank(reg >> 8);
        }
        return IMU::WriteReg(reg, val);
    }
    int ICM20948::WriteRegVerified(uint16_t reg, const uint8_t val) {
        if (bank != reg >> 8) {
            SwitchBank(reg >> 8);
        }
        return IMU::WriteRegVerified(reg, val);
    }
    int ICM20948::ReadReg(uint16_t reg, uint8_t *val) {
        if (bank != reg >> 8) {
            SwitchBank(reg >> 8);
        }
        return IMU::ReadReg(reg, val);
    }
    int ICM20948::ModifyReg(uint16_t reg, const uint8_t val, const uint8_t mask) {
        if (bank != reg >> 8) {
            SwitchBank(reg >> 8);
        }
        return IMU::ModifyReg(reg, val, mask);
    }

    int ICM20948::Init(Config _cfg) {
        cfg = _cfg;
        isOpen = false;
        int res = 0;
        if (!Detect()) { return -1; }
        //软复位
        res |= SoftReset();

        //打开所有传感器
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::PWR_MGMT_2, 0);

        //1.125khz采样率
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::GYRO_SMPLRT_DIV, 0);
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::ACCEL_SMPLRT_DIV_1, 0);
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::ACCEL_SMPLRT_DIV_2, 0);

        //配置陀螺仪lpf
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::GYRO_CONFIG_1, 1 | (ICM20948_GBW_MAP[cfg.gyroBandwidth] << 3));

        //配置陀螺仪量程和单位
        gyroUnit = MPU_GYRO_UNIT_IN_DPS[cfg.gyroFullScale] * MPU_GYRO_UNIT_FROM_DPS[cfg.gyroUnit];
        res |= ModifyReg((uint16_t) ICM20948_RegMap::GYRO_CONFIG_1, MPU_GYRO_FS_MAP[cfg.gyroFullScale] << 1, 3 << 1);

        //配置加速度计lpf
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::ACCEL_CONFIG, 1 | (ICM20948_ABW_MAP[cfg.accelBandwidth] << 3));

        //配置加速度计量程和单位
        accelUnit = MPU_ACCEL_UNIT_IN_G[cfg.accelFullScale] * MPU_ACCEL_UNIT_FROM_G[cfg.accelUnit];
        res |= ModifyReg((uint16_t) ICM20948_RegMap::ACCEL_CONFIG, MPU_ACCEL_FS_MAP[cfg.accelFullScale] << 1, 3 << 1);

        //设置温度输出lpf
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::TEMP_CONFIG, 6); //8hz

        //开启数据更新中断
        res |= EnableDataReadyInt();

        if (res != 0) { return res; }
        isOpen = false;

        //设置内部i2c
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_MST_CTRL, 1 << 4 | 12);//471khz，连续读模式
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::USER_CTRL, 1 << 5);//开启i2c主模式

        //开始设置ak8963
        res |= SubI2cRead(ICM20948_AK09916_I2C_ADDR, (uint8_t) AK09916_RegMap::WIA2, &ak09916DeviceId, 1);
        if (res != 0) { return res; }

        //复位
        uint8_t val = 0x01;
        res |= SubI2cWrite(ICM20948_AK09916_I2C_ADDR, (uint8_t) AK09916_RegMap::CNTL3, &val, 1);
        val = 0;
        res |= SubI2cWrite(ICM20948_AK09916_I2C_ADDR, (uint8_t) AK09916_RegMap::CNTL2, &val, 1);

        //设置连续读ak8963到fifo
        val = 0x5D;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_MST_CTRL, val);

        val = ICM20948_AK09916_I2C_ADDR | 0x80;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_SLV0_ADDR, val);

        val = (uint8_t) AK09916_RegMap::ST1;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_SLV0_REG, val);


        //唤醒ak09916
        val = (1 << 3);
        res |= SubI2cWrite(ICM20948_AK09916_I2C_ADDR, (uint8_t) AK09916_RegMap::CNTL2, &val, 1);

        val = 0x89;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_SLV0_CTRL, val);

        val = 0x09;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_SLV4_CTRL, val);

        val = 0x81;
        res |= WriteRegVerified((uint16_t) ICM20948_RegMap::I2C_MST_DELAY_CTRL, val);


        if (res == 0) {
            isOpen = true;
            return 0;
        } else {
            return res;
        }
    }
    int ICM20948::SelfTest() {
        if (!IsOpen()) { return -1; }
        int res = 0;
        Config backup_cfg = cfg;
        Config st_cfg;
        st_cfg.gyroFullScale = Config::MPU_FS_250dps;
        st_cfg.accelFullScale = Config::MPU_FS_2G;
        st_cfg.accelBandwidth = Config::MPU_ABW_99;
        st_cfg.gyroBandwidth = Config::MPU_GBW_92;
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
//        uint8_t val;
        memset(gyro_bias_st, 0, sizeof(gyro_bias_st));
        memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
        memset(accel_bias_st, 0, sizeof(accel_bias_st));
        memset(accel_bias_regular, 0, sizeof(accel_bias_regular));

        int times;
        times = 50;
        while (times--) { while (!DataReady()) {}}//丢弃前20个数据
        times = 50;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_regular[i] += gbuf[i];
                accel_bias_regular[i] += abuf[i];
            }
        }

        //打开自检输出
        res |= ModifyReg((uint16_t) ICM20948_RegMap::GYRO_CONFIG_2, 0b111 << 3, 0b111 << 3);
        res |= ModifyReg((uint16_t) ICM20948_RegMap::ACCEL_CONFIG_2, 0b111 << 3, 0b111 << 3);

        times = 50;
        while (times--) { while (!DataReady()) {}}//丢弃前20个数据
        times = 50;
        while (times--) {
            while (!DataReady()) {}
            res |= ReadSensorBlocking();
            Convert(abuf, abuf + 1, abuf + 2, gbuf, gbuf + 1, gbuf + 2);
            for (int i = 0; i < 3; ++i) {
                gyro_bias_st[i] += gbuf[i];
                accel_bias_st[i] += abuf[i];
            }
        }

        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] /= 50;
            accel_bias_regular[i] /= 50;
            gyro_bias_st[i] /= 50;
            accel_bias_st[i] /= 50;
        }


        //计算加速度计自检结果
        uint8_t regs[3];
        int otp_value_zero = 0;
        int st_shift_prod[3], st_shift_cust[3], i;
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_X_ACCEL, regs);
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_Y_ACCEL, regs + 1);
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_Z_ACCEL, regs + 2);
        for (i = 0; i < 3; i++) {
            if (regs[i] != 0) {
                st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
            } else {
                st_shift_prod[i] = 0;
                otp_value_zero = 1;
            }
        }

        if (!otp_value_zero) {
            for (i = 0; i < 3; i++) {
                st_shift_cust[i] = accel_bias_st[i] - accel_bias_regular[i];
                if (st_shift_cust[i] < (st_shift_prod[i] >> 1)
                    || st_shift_cust[i] > ((st_shift_prod[i] >> 1) + st_shift_prod[i])) {
                    //加速度计自检未通过
                    accel_result = 1;
                    INV_DEBUG("accel[%d] st fail,result = %f,ref:0.5<result<1.5", i, (float) st_shift_cust[i] / st_shift_prod[i]);
                } else {
                    INV_TRACE("accel[%d] st result = %f,ref:0.5<result<1.5", i, (float) st_shift_cust[i] / st_shift_prod[i]);
                }
            }
        } else {
            accel_result = 1;
            INV_DEBUG("accel[%d] st fail,otp_value=0", i);
        }

        //计算陀螺仪自检结果
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_X_GYRO, regs);
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_Y_GYRO, regs + 1);
        res |= ReadReg((uint16_t) ICM20948_RegMap::SELF_TEST_Z_GYRO, regs + 2);
        for (i = 0; i < 3; i++) {
            if (regs[i] != 0) {
                st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
            } else {
                st_shift_prod[i] = 0;
                otp_value_zero = 1;
            }
        }


        if (!otp_value_zero) {
            for (i = 0; i < 3; i++) {
                st_shift_cust[i] = gyro_bias_st[i] - gyro_bias_regular[i];
                if (st_shift_cust[i] < (st_shift_prod[i] >> 1)) {
                    //陀螺仪自检未通过
                    accel_result = 1;
                    INV_DEBUG("gyro[%d] st fail,result = %f,ref:0.5<result<1.5", i, (float) st_shift_cust[i] / st_shift_prod[i]);
                } else {
                    INV_TRACE("gyro[%d] st result = %f,ref:0.5<result<1.5", i, (float) st_shift_cust[i] / st_shift_prod[i]);
                }
            }
        } else {
            accel_result = 1;
            INV_DEBUG("gyro[%d] st fail,otp_value=0", i);
        }

        //恢复原来的配置
        res |= Init(backup_cfg);
        return res | (gyro_result << 1) | accel_result;
    }
    bool ICM20948::DataReady() {
        uint8_t val = 0;
        ReadReg((uint16_t) ICM20948_RegMap::INT_STATUS_1, &val);
        return (val & 0x01) == 0x01;
    }
    int ICM20948::EnableDataReadyInt() {
        return ModifyReg((uint16_t) ICM20948_RegMap::INT_ENABLE_1, 0x01, 0x01);
    }
    int ICM20948::SubI2cRead(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len) {
        uint8_t index = 0;
        uint8_t status = 0;
        uint32_t timeout = 0;
        uint8_t tmp = 0;
        int res = 0;
        tmp = addr | 0x80;
        res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_REG, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    INV_DEBUG("SubI2cRead time out");
                    return -2;
                }
                res |= ReadReg((uint16_t) ICM20948_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            if (status & MPU9250_I2C_SLV4_NACK) {
                INV_DEBUG("SubI2cRead no ack");
                return -3;
            }
            res |= ReadReg((uint16_t) ICM20948_RegMap::I2C_SLV4_DI, val + index);
            index++;
        }
        return res;
    }
    int ICM20948::SubI2cWrite(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len) {
        uint32_t timeout = 0;
        uint8_t status = 0;
        uint8_t tmp = 0;
        uint8_t index = 0;
        int res = 0;
        tmp = addr;
        res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_ADDR, tmp);
        while (index < len) {
            tmp = reg + index;
            res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_REG, tmp);
            tmp = val[index];
            res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_DO, tmp);
            tmp = MPU9250_I2C_SLV4_EN;
            res |= WriteReg((uint16_t) ICM20948_RegMap::I2C_SLV4_CTRL, tmp);
            do {
                if (timeout++ > 5000) {
                    INV_DEBUG("SubI2cWrite time out");
                    return -2;
                }
                res |= ReadReg((uint16_t) ICM20948_RegMap::I2C_MST_STATUS, &status);
            } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
            if (status & MPU9250_I2C_SLV4_NACK) {
                INV_DEBUG("SubI2cWrite no ack");
                return -3;
            }
            index++;
        }
        return res;
    }

    int ICM20948::ReadSensorBlocking() {
        if (bank != 0) {
            SwitchBank(0);
        }
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) ICM20948_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 23;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) ICM20948_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 24;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int ICM20948::ReadSensorNonBlocking() {
        if (bank != 0) {
            SwitchBank(0);
        }
        int res;
        if (i2c != nullptr) {
            i2cTransfer.subAddress = (uint8_t) ICM20948_RegMap::ACCEL_XOUT_H;
            i2cTransfer.data = buf;
            i2cTransfer.dataSize = 23;
            i2cTransfer.direction = I2C::Transfer::Read;
            res = i2c->masterTransferNonBlocking(i2cTransfer);
            if (res != 0) {
                INV_DEBUG("i2c read return code = %d", res);
            }
        } else {
            txbuf[0] = (1U << 7U) | ((uint8_t) ICM20948_RegMap::ACCEL_XOUT_H & 0x7fU);
            spiTransfer.dataSize = 24;
            spiTransfer.rxData = rxbuf;
            spiTransfer.txData = txbuf;
            res = spi->masterTransferNonBlocking(spiTransfer);
            if (res != 0) {
                INV_DEBUG("spi read return code = %d", res);
            }
        }
        return res;
    }

    int ICM20948::Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) {
        if (acc_x) { *acc_x = accelUnit * ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = accelUnit * ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = accelUnit * ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = gyroUnit * ((int16_t) ((buf[6] << 8) | buf[7])); }
        if (gyro_y) { *gyro_y = gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_z) { *gyro_z = gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11])); }
        return 0;
    }
    int ICM20948::Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
        if (acc_x) { *acc_x = ((int16_t) ((buf[0] << 8) | buf[1])); }
        if (acc_y) { *acc_y = ((int16_t) ((buf[2] << 8) | buf[3])); }
        if (acc_z) { *acc_z = ((int16_t) ((buf[4] << 8) | buf[5])); }
        if (gyro_x) { *gyro_x = ((int16_t) ((buf[6] << 8) | buf[7])); }
        if (gyro_y) { *gyro_y = ((int16_t) ((buf[8] << 8) | buf[9])); }
        if (gyro_z) { *gyro_z = ((int16_t) ((buf[10] << 8) | buf[11])); }
        return 0;
    }

    int ICM20948::Convert(float *mag_x, float *mag_y, float *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
//        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY)) {
//            INV_TRACE("0x%x at buf[14 + 0]", (int) buf[14 + 0]);
            return -1;
        }
        if (buf[14 + 8] & MPU9250_AK8963_OVERFLOW) {
//            INV_TRACE("0x%x at buf[14 + 7]", (int) buf[14 + 7]);
            return -1;
        }
        if (mag_x) { *mag_x = magUnit * ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = magUnit * ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = magUnit * ((int16_t) (buf[14 + 6] << 8) | buf[14 + 5]); }
        return 0;
    }
    int ICM20948::Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        if (!(buf[14 + 0] & MPU9250_AK8963_DATA_READY) || (buf[14 + 0] & MPU9250_AK8963_DATA_OVERRUN)) {
//            INV_TRACE("0x%x at buf[14 + 0]", (int) buf[14 + 0]);
            return -1;
        }
        if (buf[14 + 8] & MPU9250_AK8963_OVERFLOW) {
//            INV_TRACE("0x%x at buf[14 + 7]", (int) buf[14 + 7]);
            return -1;
        }
        if (mag_x) { *mag_x = ((int16_t) (buf[14 + 2] << 8) | buf[14 + 1]); }
        if (mag_y) { *mag_y = ((int16_t) (buf[14 + 4] << 8) | buf[14 + 3]); }
        if (mag_z) { *mag_z = ((int16_t) (buf[14 + 6] << 8) | buf[14 + 5]); }
        return 0;
    }
    int ICM20948::Convert(float *temp) {
        if (temp) { *temp = (float) ((int16_t) ((buf[12] << 8) | buf[13])) / 333.87f + 21; }
        return 0;
    }

    std::string ICM20948::Report() {
        std::string rtv;
        rtv += "model:icm20948\t";
        rtv += "addr:";
        rtv += std::to_string((int) i2cTransfer.slaveAddress);
        rtv += '\t';

        rtv += "magnetometer:ak09916\t";
        rtv += "ID:";
        rtv += std::to_string((int) ak09916DeviceId);
        return rtv;
    }

    int QMC5883L::Init(Config _cfg) {
        enum CONTROL1_BITSET {
            Mode_Standby = 0b00000000,
            Mode_Continuous = 0b00000001,
            ODR_10Hz = 0b00000000,
            ODR_50Hz = 0b00000100,
            ODR_100Hz = 0b00001000,
            ODR_200Hz = 0b00001100,
            RNG_2G = 0b00000000,
            RNG_8G = 0b00010000,
            OSR_512 = 0b00000000,
            OSR_256 = 0b01000000,
            OSR_128 = 0b10000000,
            OSR_64 = 0b11000000
        };
        int res = 0;
        res |= WriteRegVerified(static_cast<uint8_t>(QMC5883L_RegMap::SET_RESET_PERIOD), 0x01);
        res |= WriteRegVerified(static_cast<uint8_t>(QMC5883L_RegMap::CONTROL1), OSR_512 | RNG_8G | ODR_200Hz | Mode_Continuous);
        res |= EnableDataReadyInt();
        return res;
    }
    bool QMC5883L::Detect() {
        uint8_t val = 0xFF;
        if (0 == ReadReg(static_cast<uint8_t>(QMC5883L_RegMap::CONTROL2), &val)) {
            if (val != 0xFF) {
                return true;
            }
        }
        return false;
    }
    int QMC5883L::SelfTest() {
        return 0;
    }
    std::string QMC5883L::Report() {
        return std::string("model:QMC5883L\t");
    }
    bool QMC5883L::DataReady() {
        return true;
    }
    int QMC5883L::EnableDataReadyInt() {
        return WriteRegVerified(static_cast<uint8_t>(QMC5883L_RegMap::CONTROL2), 0x0);
    }
    int QMC5883L::SoftReset() {
        return WriteRegVerified(static_cast<uint8_t>(QMC5883L_RegMap::CONTROL2), 0x80);
    }
    int QMC5883L::ReadSensorBlocking() {
        i2cTransfer.direction = I2C::Transfer::Read;
        i2cTransfer.subAddress = (uint8_t) QMC5883L_RegMap::XOUT_L;
        i2cTransfer.data = buf;
        i2cTransfer.dataSize = 9;
        return i2c->masterTransferBlocking(i2cTransfer);
    }
    int QMC5883L::ReadSensorNonBlocking() {
        i2cTransfer.direction = I2C::Transfer::Read;
        i2cTransfer.subAddress = (uint8_t) QMC5883L_RegMap::XOUT_L;
        i2cTransfer.data = buf;
        i2cTransfer.dataSize = 9;
        return i2c->masterTransferNonBlocking(i2cTransfer);
    }
    int QMC5883L::Convert(float *acc_x, float *acc_y, float *acc_z, float *gyro_x, float *gyro_y, float *gyro_z) {
        return 0;
    }
    int QMC5883L::Convert(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
        return 0;
    }
    int QMC5883L::Convert(float *mag_x, float *mag_y, float *mag_z) {
        constexpr uint8_t DOR = 0b100;//高有效，data skipped for reading
        constexpr uint8_t OVL = 0b010;//低有效
        constexpr uint8_t DRDY = 0b001;//高有效
        if ((buf[6] & (DOR | DRDY)) && ((~buf[6]) & OVL)) {
            if (mag_x) { *mag_x = magUnit * ((int16_t) (buf[1] << 8) | buf[0]); }
            if (mag_y) { *mag_y = magUnit * ((int16_t) (buf[3] << 8) | buf[2]); }
            if (mag_z) { *mag_z = magUnit * ((int16_t) (buf[5] << 8) | buf[4]); }
            return 0;
        }
        return buf[6];
    }
    int QMC5883L::Convert(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
        constexpr uint8_t DOR = 0b100;//高有效，data skipped for reading
        constexpr uint8_t OVL = 0b010;//低有效
        constexpr uint8_t DRDY = 0b001;//高有效
        if ((buf[6] & (DOR | DRDY)) && ((~buf[6]) & OVL)) {
            if (mag_x) { *mag_x = ((int16_t) (buf[1] << 8) | buf[0]); }
            if (mag_y) { *mag_y = ((int16_t) (buf[3] << 8) | buf[2]); }
            if (mag_z) { *mag_z = ((int16_t) (buf[5] << 8) | buf[4]); }
            return 0;
        }
        return buf[6];
    }
    int QMC5883L::Convert(float *temp) {
        if (temp) { *temp = 0.01f * ((int16_t) (buf[8] << 8) | buf[7]); }
        return 0;
    }
}