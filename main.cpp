#include <iostream>
#include "remote_i2c.h"
#include "drv_imu_invensense.hpp"

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



int example1(int argc, const char **argv) {
    inv::imuPtr_t my_imu;
    if (0 == my_imu.Load(my_i2c)) {
        if (my_imu->Init() == 0) {
            //自检时保持静止，否则会直接失败
            if (my_imu->SelfTest() == 0) {
                usleep(10000);//等待10ms
                my_imu->ReadSensorBlocking();
                my_imu->Converter(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
//                static_cast<inv::mpuSeries_t*>(my_imu.get())->Converter(&temp);
                my_imu->Converter(mag, mag + 1, mag + 2);
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
                my_imu.Converter(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
                my_imu.Converter(&temp);
                my_imu.Converter(mag, mag + 1, mag + 2);
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



int main(int argc, const char **argv){
    printf("\r\n*****************example1*****************\r\n");
    example1(argc,argv);
    printf("\t\n*****************example2*****************\r\n");
    example2(argc,argv);
    printf("Hello\r\n");

    return 0;
}
