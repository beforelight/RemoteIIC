#include <iostream>
#include "remote_i2c.h"
#include "drv_imu_invensense.hpp"
#include <sys/time.h>
#include "ahrs/ahrs.h"
#include <cmath>
remote_i2c iic("/dev/i2c-1");
using namespace std;
using namespace ahrs;

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

inv::i2cInterface_t my_i2c(&iic, remote_i2c_read, remote_i2c_write);

float acc[3] = {0, 0, 0};
float gyro[3] = {0, 0, 0};
float mag[3] = {0, 0, 0};
float temp = 0;


int usGet(void) {
    struct timeval tic;
    gettimeofday(&tic, NULL);
    return tic.tv_usec + tic.tv_sec * 1000 * 1000;
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
//                static_cast<inv::mpuSeries_t*>(my_imu.get())->Convert(&temp);
                my_imu->Convert(mag, mag + 1, mag + 2);
                printf("%s\r\n", my_imu->Report().c_str());
                printf("accel \t%.3f \t%.3f \t%.3f m/s^2\r\n", acc[0], acc[1], acc[2]);
                printf("gyro \t%.3f \t%.3f \t%.3f dps \r\n", gyro[0], gyro[1], gyro[2]);
                printf("mag \t%.1f \t%.1f \t%.1f uT \r\n", mag[0], mag[1], mag[2]);
//                printf("temp \t%.3f C \r\n", temp);
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

int example3(int argc, const char **argv) {
    inv::imuPtr_t my_imu;
    if (0 == my_imu.Load(my_i2c)) {
        if (my_imu->Init() == 0) {
            //自检时保持静止，否则会直接失败
            if (my_imu->SelfTest() == 0) {
                int us_p, us_n;
                us_n = usGet();
                int times = 100;
                while (times--) {
                    while (!static_cast<inv::mpuSeries_t *>(my_imu.get())->DataReady()) {
//                        my_imu->ReadSensorBlocking();
                    }
                }
                us_p = usGet();
                printf("avg %dus\r\n", (us_p - us_n) / 100);

                us_n = usGet();
                times = 100;
                while (times--) {
                    while (!static_cast<inv::mpuSeries_t *>(my_imu.get())->DataReady()) {
                        my_imu->ReadSensorBlocking();
                    }
                }
                us_p = usGet();
                printf("avg %dus\r\n", (us_p - us_n) / 100);

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

std::ostream &operator<<(std::ostream &os, ahrs::MAGE &b) {
    os
            << b[0] << '\t' << b[3] << '\t' << b[6] << std::endl
            << b[1] << '\t' << b[4] << '\t' << b[7] << std::endl
            << b[2] << '\t' << b[5] << '\t' << b[8];
    return os;
}
int example4(int argc, const char **argv) {
    for (int i = 0; i < argc; i++) {
        cout << argv[i] << endl;
    }
    if (argc <= 1) {
        printf("st \t 对陀螺仪自检\r\n");
        printf("c \t 读取100组数据进行校准\r\n");
        printf("MM \t Mahony 和Madgwick方法融合并显示对比，请手动结束程序\r\n");
        printf("mm \t 输出传感器数据，请手动结束程序\r\n");
        return 0;
    } else {
        inv::imuPtr_t my_imu;
        MAGE avg_bias;
        //对陀螺仪自检
        if (strcmp(argv[1], "st") == 0) {
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init()) && 0 == my_imu->SelfTest()) {
                cout << "自检通过" << my_imu->Report() << endl;
            }
        }

        //读取100组数据进行校准
        if (strcmp(argv[1], "c") == 0) {
            MAGE imuData;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                int times = 100;
                int ustime = usGet();
                avg_bias.setZero();
                while (times--) {
                    while ((usGet() - ustime) < 10 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData[0 + 3 * 1],
                                    &imuData[1 + 3 * 1],
                                    &imuData[2 + 3 * 1],
                                    &imuData[0 + 3 * 0],
                                    &imuData[1 + 3 * 0],
                                    &imuData[2 + 3 * 0]);
                    my_imu->Convert(&imuData[0 + 3 * 2],
                                    &imuData[1 + 3 * 2],
                                    &imuData[2 + 3 * 2]);
                    imuData *= 0.01;
                    avg_bias += imuData;
                }
                cout << avg_bias << endl;
                cout << sqrt(avg_bias[3] * avg_bias[4] + avg_bias[4] * avg_bias[4] + avg_bias[5] * avg_bias[5]) << endl;
            }
        }

        //MM方法融合并且对比，请手动结束程序
        if (strcmp(argv[1], "MM") == 0) {
            MAGE imuData;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                Mahony myAHRS(0.02);
                Madgwick myAHRS2(0.02);
                cout << avg_bias << endl;
                int ustime = usGet();
                imuData.setZero();
                while (1) {
                    while ((usGet() - ustime) < 20 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData[0 + 3 * 1],
                                    &imuData[1 + 3 * 1],
                                    &imuData[2 + 3 * 1],
                                    &imuData[0 + 3 * 0],
                                    &imuData[1 + 3 * 0],
                                    &imuData[2 + 3 * 0]);
                    my_imu->Convert(&imuData[0 + 3 * 2],
                                    &imuData[1 + 3 * 2],
                                    &imuData[2 + 3 * 2]);
                    imuData -= avg_bias;
                    imuData[0]*=M_PI/180;
                    imuData[1]*=M_PI/180;
                    imuData[2]*=M_PI/180;
                    myAHRS.update(imuData);
                    myAHRS2.update(imuData);
                    float rpy[3];
                    float rpy2[3];
                    myAHRS.eulerAngle(rpy[0],rpy[1],rpy[2]);
                    myAHRS2.eulerAngle(rpy2[0],rpy2[1],rpy2[2]);
                    for (int i = 0; i < 3; ++i) {
                        rpy[i]*=180/M_PI;
                        rpy2[i]*=180/M_PI;
                    }
//                    Vector3 buf = (180 / 3.14159) * eulerAngle(myAHRS.quaternion());
//                    Vector3 buf2 = (180 / 3.14159) * eulerAngle(myAHRS2.quaternion());
                    printf("RPY Mahony vs Madgwick\t %6.2f %6.2f\t|\t%6.2f %6.2f\t|\t%6.2f %6.2f\r\n",
                           rpy[0], rpy2[0],
                           rpy[1], rpy2[1],
                           rpy[2], rpy2[2]
                    );
                }
            }
        }

        //输出传感器数据，请手动结束程序
        if (strcmp(argv[1], "mm") == 0) {
            MAGE imuData;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                int ustime = usGet();
                imuData.setZero();
                while (1) {
                    while ((usGet() - ustime) < 20 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData[0 + 3 * 1],
                                    &imuData[1 + 3 * 1],
                                    &imuData[2 + 3 * 1],
                                    &imuData[0 + 3 * 0],
                                    &imuData[1 + 3 * 0],
                                    &imuData[2 + 3 * 0]);
                    my_imu->Convert(&imuData[0 + 3 * 2],
                                    &imuData[1 + 3 * 2],
                                    &imuData[2 + 3 * 2]);
                    printf("gam xyz|\t%6.3f %6.3f %6.3f\t|\t%6.3f %6.3f %6.3f\t|\t%7.2f %7.2f %7.2f\r\n",
                           imuData[0 + 3 * 0],
                           imuData[1 + 3 * 0],
                           imuData[2 + 3 * 0],
                           imuData[0 + 3 * 1],
                           imuData[1 + 3 * 1],
                           imuData[2 + 3 * 1],
                           imuData[0 + 3 * 2],
                           imuData[1 + 3 * 2],
                           imuData[2 + 3 * 2]
                    );
                }
            }
        }
        return 0;
    }
    return 0;
}
#include "vector"
int main(int argc, const char **argv) {
    vector<float> a = {1, 2, 3, 4, 5};
    a = {1, 2, 3, 4, 5, 6};
//    printf("\r\n*****************example1*****************\r\n");
//    example1(argc, argv);
//    printf("\t\n*****************example2*****************\r\n");
//    example2(argc, argv);
//    printf("\t\n*****************example3*****************\r\n");
    example4(argc, argv);
    printf("Hello\r\n");

    return 0;
}
