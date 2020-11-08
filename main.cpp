#include <iostream>
#include "remote_i2c.h"
#include "drv_imu_invensense.hpp"
#include <sys/time.h>
#include "ahrs/ahrs.h"
remote_i2c iic("/dev/i2c-1");
using namespace std;
using namespace ahrs;

inv::i2cInterface_t my_i2c(
        [](unsigned int addr, unsigned int reg, unsigned char *buf, unsigned int len) -> int {
            return iic.Read(addr, reg, buf, len);
        },
        [](unsigned int addr, unsigned int reg, const unsigned char *buf, unsigned int len) -> int {
            return iic.Write(addr, reg, buf, len);
        });

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

        Matrix3x2 avg;
//        avg << 0.69519, -4.43948,
//                -1.60522, 3.21361,
//                0.372314, -8.57572;
        avg << 0.69519, 0,
                -1.60522, 0,
                0.372314, 9.4;
        Matrix3 Sen2Obj;


        //对陀螺仪自检
        if (strcmp(argv[1], "st") == 0) {
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init()) && 0 == my_imu->SelfTest()) {
                cout << "自检通过" << my_imu->Report() << endl;
            }
        }

        //读取100组数据进行校准
        if (strcmp(argv[1], "c") == 0) {
            Matrix3x2 imuData;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                int times = 100;
                int ustime = usGet();
                avg.setZero();
                while (times--) {
                    while ((usGet() - ustime) < 10 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData(0, 1),
                                    &imuData(1, 1),
                                    &imuData(2, 1),
                                    &imuData(0, 0),
                                    &imuData(1, 0),
                                    &imuData(2, 0));
                    avg += imuData * 0.01;
                }
                cout << avg << endl;
                cout << avg.block(0, 1, 3, 1).norm() << endl;
            }
        }

        //Mahony方法融合，请手动结束程序
        if (strcmp(argv[1], "MM") == 0) {
            Matrix3 imuData;
            Matrix3 imuDataObject;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                Mahony myAHRS(0.02);
                Madgwick myAHRS2(0.02);
                Sen2Obj = Sensor2ObjectRotationMatrix(avg.block(0, 1, 3, 1));
                cout << avg << endl;
                cout << Sen2Obj << endl;
                Sen2Obj.setIdentity();
                int ustime = usGet();
                imuData.setZero();
                while (1) {
                    while ((usGet() - ustime) < 20 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData(0, 1),
                                    &imuData(1, 1),
                                    &imuData(2, 1),
                                    &imuData(0, 0),
                                    &imuData(1, 0),
                                    &imuData(2, 0));
                    my_imu->Convert(&imuData(0, 2),
                                    &imuData(1, 2),
                                    &imuData(2, 2));
                    imuData.block(0, 0, 3, 1)
                            -= avg.block(0, 0, 3, 1);
                    imuData.block(0, 0, 3, 1)
                            = ((3.1415926535 / 180) * Matrix3::Identity()) * imuData.block(0, 0, 3, 1);

                    imuDataObject = Sen2Obj * imuData;
                    myAHRS.update(imuDataObject);
                    myAHRS2.update(imuDataObject);
                    Vector3 buf = (180 / 3.14159) * eulerAngle(myAHRS.quaternion());
                    Vector3 buf2 = (180 / 3.14159) * eulerAngle(myAHRS2.quaternion());
                    printf("RPY Mahony vs Madgwick\t %6.1f %6.1f\t|\t%6.1f %6.1f\t|\t%6.1f %6.1f\r\n",
                           buf.x(), buf2.x(),
                           buf.y(), buf2.y(),
                           buf.z(), buf2.z()
                    );
                }
            }
        }

        //输出传感器数据，请手动结束程序
        if (strcmp(argv[1], "mm") == 0) {
            Matrix3 imuData;
            if (0 == my_imu.Load(my_i2c) && (0 == my_imu->Init())) {
                int ustime = usGet();
                imuData.setZero();
                while (1) {
                    while ((usGet() - ustime) < 20 * 1000) {}
                    ustime = usGet();
                    my_imu->ReadSensorBlocking();
                    my_imu->Convert(&imuData(0, 1),
                                    &imuData(1, 1),
                                    &imuData(2, 1),
                                    &imuData(0, 0),
                                    &imuData(1, 0),
                                    &imuData(2, 0));
                    my_imu->Convert(&imuData(0, 2),
                                    &imuData(1, 2),
                                    &imuData(2, 2));
                    printf("gam xyz|\t%6.1f %6.1f %6.1f\t|\t%6.1f %6.1f %6.1f\t|\t%6.1f %6.1f %6.1f\r\n",
                           imuData(0, 0),
                           imuData(1, 0),
                           imuData(2, 0),
                           imuData(0, 1),
                           imuData(1, 1),
                           imuData(2, 1),
                           imuData(0, 2),
                           imuData(1, 2),
                           imuData(2, 2)
                    );
                }
            }
        }
        return 0;
    }
    return 0;
}

int main(int argc, const char **argv) {
//    printf("\r\n*****************example1*****************\r\n");
//    example1(argc, argv);
//    printf("\t\n*****************example2*****************\r\n");
//    example2(argc, argv);
//    printf("\t\n*****************example3*****************\r\n");
    example4(argc, argv);
    printf("Hello\r\n");

    return 0;
}
