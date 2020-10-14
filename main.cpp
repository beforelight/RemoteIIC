#include <iostream>
#include"remote_i2c.h"
#include"inv_imu.h"

int remote_i2c_read(void *context,
                    unsigned char addr, unsigned char reg,
                    unsigned char *val, unsigned int len) {
    return static_cast<remote_i2c *>(context)->Read(addr, reg, val, len);
}

int remote_i2c_write(void *context,
                     unsigned char addr, unsigned char reg,
                     const unsigned char *val, unsigned int len) {
    return static_cast<remote_i2c *>(context)->Write(addr, reg, val, len);
}

remote_i2c iic("/dev/i2c-1");
inv::i2c_interface my_i2c(&iic, remote_i2c_read, remote_i2c_write,
                          remote_i2c_read, remote_i2c_write);
std::shared_ptr<inv::imu> my_imu;
uint8_t val;

float acc[3] = {0, 0, 0};
float gyro[3] = {0, 0, 0};
float mag[3] = {0, 0, 0};
float temp = 0;

int main(int argc, const char **argv) {
    if (0 == inv::Parser(my_i2c, my_imu)) {
        if (my_imu->init() == 0) {
            //自检时保持静止，否则会直接失败
            if (my_imu->self_test() == 0) {
                usleep(10000);//等待10ms
                my_imu->read_sensor_blocking();
                my_imu->converter(acc, acc + 1, acc + 2, gyro, gyro + 1, gyro + 2);
                my_imu->converter(&temp);
                my_imu->converter(mag, mag + 1, mag + 2);
                printf("%s\r\n", my_imu->report().c_str());
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
    printf("Hello\r\n");
    return 0;
}

