#include <iostream>
#include "remote_i2c.h"
#include "inv_imu.hpp"
#include <map>
#include <cmath>
using namespace std;

int remote_i2c_read(void *context,
                    uint8_t addr, uint8_t reg,
                    uint8_t *val, unsigned int len) {
    return static_cast<remote_i2c *>(context)->Read(addr, reg, val, len);
}

int remote_i2c_write(void *context,
                     uint8_t addr, uint8_t reg,
                     const uint8_t *val, unsigned int len) {
    return static_cast<remote_i2c *>(context)->Write(addr, reg, val, len);
}
remote_i2c iic("/dev/i2c-1");
inv::i2cInterface_t my_i2c(&iic, remote_i2c_read, remote_i2c_write);

float acc[3] = {0, 0, 0};
float gyro[3] = {0, 0, 0};
float mag[3] = {0, 0, 0};
float temp = 0;


int imu_invensense_example(int argc, const char **argv);

int main(int argc, const char **argv) {
    imu_invensense_example(argc, argv);
    printf("\r\n");
    return 0;
}

int imu_invensense_example(int argc, const char **argv) {
    printf("*****************imu_invensense_example*****************\r\n");
    if (argc <= 1) {
        printf("-m [module] \t所测试的传感器\r\n");
        printf("-n num      \t使用哪个i2c\r\n");
        printf("-t times    \t采集多少次数据，默认0\r\n");
        printf("-dt ms      \t采集周期\r\n");
        printf("-nost       \t不自检\r\n");
        printf("-h          \t帮助信息\r\n");
    }
    int i2c_num = 1;
    uint32_t times = 0;
    uint32_t dt = 10;
    int nost = 0;
    inv::imuPtr_t my_imu;
    std::map<std::string, inv::imuPtr_t> map_imu;
    map_imu["mpu6050"].reset(new inv::mpu6050_t(my_i2c));
    map_imu["mpu9250"].reset(new inv::mpu9250_t(my_i2c));
    map_imu["icm20602"].reset(new inv::icm20602_t(my_i2c));
    map_imu["icm20600"].reset(new inv::icm20600_t(my_i2c));
    map_imu["icm20948"].reset(new inv::icm20948_t(my_i2c));

    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-m") == 0) {
            if (++i < argc) {
                for (auto j:map_imu) {
                    if (j.first.find(argv[i]) != std::string::npos) {
                        my_imu = j.second;
                    }
                }
            }
            continue;
        }
        if (strcmp(argv[i], "-n") == 0) {
            if (++i < argc) {
                i2c_num = atoi(argv[i]);
                string bus = "/dev/i2c";
                bus += to_string(i2c_num);
                iic.Open(bus.c_str());
            }
            continue;
        }
        if (strcmp(argv[i], "-t") == 0) {
            if (++i < argc) {
                times = atoi(argv[i]);
            }
            continue;
        }
        if (strcmp(argv[i], "-dt") == 0) {
            if (++i < argc) {
                dt = atoi(argv[i]);
            }
            continue;
        }
        if (strcmp(argv[i], "-nost") == 0) {
            nost = 1;
            continue;
        }
        if (strcmp(argv[i], "-h") == 0) {
            printf("-m [module] \t所测试的传感器\r\n");
            printf("-n num      \t使用哪个i2c\r\n");
            printf("-t times    \t采集多少次数据，默认0\r\n");
            printf("-dt ms      \t采集周期\r\n");
            printf("-nost       \t不自检\r\n");
            printf("-h          \t帮助信息\r\n");
            return 0;
            continue;
        }
    }

    if (times) {
        if (!my_imu) {
            my_imu.Load(my_i2c);
        }
        if (my_imu->Detect()) {
            if (my_imu->Init() == 0) {
                cout << my_imu->Report() << endl;
                //自检时保持静止，否则会直接失败
                if (nost || my_imu->SelfTest() == 0) {
                    while (times--) {
                        usleep(1000 * dt);
                        my_imu->ReadSensorBlocking();
                        my_imu->Convert(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
                        my_imu->Convert(&temp);
                        my_imu->Convert(mag, mag + 1, mag + 2);
                        printf("dps| m/s^2| uT|° g m\t %8.3f %8.3f %8.3f\t| %8.3f %8.3f %8.3f\t| %8.3f %8.3f %8.3f\t| %8.3f %8.3f %8.3f\r\n",
                               gyro[0], gyro[1], gyro[2],
                               acc[0], acc[1], acc[2],
                               mag[0], mag[1], mag[2],
                               temp,
                               sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]),
                               sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2])
                        );
                    }
                } else {
                    printf("自检失败\r\n");
                }
            } else {
                printf("初始化失败\r\n");
            }
        } else {
            printf("无imu\r\n");
        }
    }
    return 0;
}


int example1(int argc, const char **argv) {
    inv::imuPtr_t my_imu;
    if (0 == my_imu.Load(my_i2c)) {
        if (my_imu->Init() == 0) {
            //自检时保持静止，否则会直接失败
            if (my_imu->SelfTest() == 0) {
                usleep(10000);//等待10ms
                my_imu->ReadSensorBlocking();
                my_imu->Convert(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
                my_imu->Convert(&temp);
                my_imu->Convert(mag, mag + 1, mag + 2);
                printf("%s\r\n", my_imu->Report().c_str());
                printf("accel \t%.3f \t%.3f \t%.3f m/s^2\r\n", acc[0], acc[1], acc[2]);
                printf("gyro \t%.3f \t%.3f \t%.3f dps \r\n", gyro[0], gyro[1], gyro[2]);
                printf("mag \t%.1f \t%.1f \t%.1f uT \r\n", mag[0], mag[1], mag[2]);
                printf("temp \t%.3f C \r\n", temp);
            } else {
                printf("自检未通过\r\n");
            }
        } else {
            printf("初始化未通过\r\n");
        }
    } else {
        printf("没有imu\r\n");
    }
    return 0;
}

int example2(int argc, const char **argv) {
    inv::mpu6050_t my_imu(my_i2c);
    if (true == my_imu.Detect()) {
        if (my_imu.Init() == 0) {
            //自检时保持静止，否则会直接失败
            if (my_imu.SelfTest() == 0) {
                usleep(10000);//等待10ms
                my_imu.ReadSensorBlocking();
                my_imu.Convert(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
                my_imu.Convert(&temp);
                my_imu.Convert(mag, mag + 1, mag + 2);
                printf("%s\r\n", my_imu.Report().c_str());
                printf("accel \t%.3f \t%.3f \t%.3f m/s^2\r\n", acc[0], acc[1], acc[2]);
                printf("gyro \t%.3f \t%.3f \t%.3f dps \r\n", gyro[0], gyro[1], gyro[2]);
                printf("mag \t%.1f \t%.1f \t%.1f uT \r\n", mag[0], mag[1], mag[2]);
                printf("temp \t%.3f C \r\n", temp);
            } else {
                printf("自检未通过\r\n");
            }
        } else {
            printf("初始化未通过\r\n");
        }
    } else {
        printf("没有mpu6050\r\n");
    }
    return 0;
}