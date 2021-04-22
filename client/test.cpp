//
// Created by 17616 on 2021/4/21.
//
#include <windows.h>
#include"client.hpp"
#include<stdio.h>
#define TEST_GPIO 1
#define TEST_I2C 1
#define TEST_SPI 0
#include"c_inv_imu/drv_imu_invensense.h"

void test_iic();
void test_spi();
client_t clt;
int main() {
    clt.Init(4);


    if (TEST_I2C) {
        test_iic();
    }
    if (TEST_SPI) {
        test_spi();
    }
    while (TEST_GPIO) {
        clt.GPIO_Write(0, 1);
        Sleep(1000);
        clt.GPIO_Write(0, 0);
        Sleep(1000);
        printf("io = %d\r\n",clt.GPIO_Read(3));
    }
    return 0;
}
int IMU_INV_I2C_TransferBlocking(const inv_i2c_transfer_t *tf) {
    client_t::inv_i2c_transfer_t tf2;
    tf2.dataSize = tf->dataSize;
    tf2.data = tf->data;
    tf2.direction = (client_t::inv_i2c_direction_t) tf->direction;
    tf2.subAddressSize = tf->subAddressSize;
    tf2.subAddress = tf->subAddress;
    tf2.slaveAddress = tf->slaveAddress;
    tf2.i2cx = 1;
    int res = clt.I2C_Transfer(tf2);
    return res;
}
void test_iic() {
    inv_i2c_t i2c0;
    i2c0.masterTransferBlocking = IMU_INV_I2C_TransferBlocking;
    i2c0.masterTransferNonBlocking = IMU_INV_I2C_TransferBlocking;
    inv_imu_handle_t imu0 = IMU_AutoConstructI2C(i2c0, IMU_SlaveAddressAutoDetect);
    //or inv_imu_handle imu0 = IMU_AutoConstructSPI(spi0);
    if (imu0 != NULL) {
        printf("当前使用的传感器为 %s", IMU_Report(imu0));
        if (0 == IMU_Init(imu0, IMU_ConfigDefault())) {
            printf("ST = %d,0为自检通过,自检时保持静止", IMU_SelfTest(imu0));
            INV_DELAY(10);
            int loop = 10;
            while (loop--) {
                IMU_ReadSensorBlocking(imu0);
                float temp;
                float data[9];
                float *buf = data;
                IMU_Convert(imu0, data);
                IMU_ConvertTemp(imu0, &temp);
                printf("temp = %f", temp);
                printf("accel(xyz) = %.3f %.3f %.3f\t gyro(xyz) = %.3f %.3f %.3f\t mag(xyz) = %.3f %.3f %.3f \r\n",
                         *buf++, *buf++, *buf++, *buf++, *buf++, *buf++, *buf++, *buf++, *buf++);
            }
        } else {
            printf("初始化失败");
        }
    } else {
        printf("imu0 == NULL，没接或者iic/spi读写出错");
    }
    if (imu0 != NULL) {
        printf("释放资源");
        IMU_Destruct(imu0);
    }
}

void test_spi(){

}