//
// Created by 17616 on 2021/4/21.
//

#include "conf.h"
#include<vector>
#include <usbd_cdc_if.h>
#include"stm32_hal.hpp"
#include "gpio.hpp"
#include "main.h"
#include<assert.h>
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

std::vector<packageProcess> cmd_map_process = {
        ProcessGpioPackage,//{write_gpio
        ProcessGpioPackage,//{read_gpio,
        ProcessI2cPackage,//{write_i2c,
        ProcessI2cPackage,//{read_i2c,
        ProcessSpiPackage,//{write_spi,
        ProcessSpiPackage,//{read_spi,
        ProcessSpiPackage,//{transfer_spi,
};
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
std::vector<I2C_HandleTypeDef *> num_map_i2c = {
        NULL,
        &hi2c1,
};
std::vector<SPI_HandleTypeDef *> num_map_spi = {
        NULL,
        &hspi1,
        &hspi2,
};
std::vector<GPIO> num_map_gpio = {
        GPIO(GPIO0_GPIO_Port, GPIO0_Pin),
        GPIO(GPIO1_GPIO_Port, GPIO1_Pin),
        GPIO(GPIO2_GPIO_Port, GPIO2_Pin),
        GPIO(GPIO3_GPIO_Port, GPIO3_Pin),
        GPIO(GPIO4_GPIO_Port, GPIO4_Pin),
        GPIO(GPIO5_GPIO_Port, GPIO5_Pin),
        GPIO(GPIO6_GPIO_Port, GPIO6_Pin),
        GPIO(GPIO7_GPIO_Port, GPIO7_Pin),
        GPIO(GPIO8_GPIO_Port, GPIO8_Pin),
        GPIO(GPIO9_GPIO_Port, GPIO9_Pin),
        GPIO(GPIO10_GPIO_Port, GPIO10_Pin),
        GPIO(GPIO11_GPIO_Port, GPIO11_Pin),
        GPIO(GPIO12_GPIO_Port, GPIO12_Pin),
        GPIO(GPIO13_GPIO_Port, GPIO13_Pin),
        GPIO(GPIO14_GPIO_Port, GPIO14_Pin),
        GPIO(GPIO15_GPIO_Port, GPIO15_Pin),
        GPIO(GPIO16_GPIO_Port, GPIO16_Pin),
        GPIO(GPIO17_GPIO_Port, GPIO17_Pin),
        GPIO(GPIO18_GPIO_Port, GPIO18_Pin),
        GPIO(GPIO19_GPIO_Port, GPIO19_Pin),
};

//extern "C" {
int CopeReceivedPackage(package_t *pack) {
    if (packageVerify(pack)) {
        return cmd_map_process[pack->cmd](pack);
    } else {
        uint8_t *buf = (uint8_t *) pack;
        printf("crc错误 cmd:0x%x length:0x%x crc:0x%x\r\n", pack->cmd, pack->length, buf[pack->length]);
        assert(0);
    }
    return 0;
}
//}
int ProcessGpioPackage(package_t *pack) {
//    write+gpio?+val //无需响应
//    read+gpio?
//    re|read+val
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    package_t *packToSend = (package_t *) UserTxBufferFS;
    uint8_t *bufToSend = (uint8_t *) packToSend;
    bufToSend += sizeof(package_t);
    uint8_t val;

    if (pack->cmd == write_gpio) {
        num_map_gpio[buf[0]].Write(buf[1]);
    } else {
        val = num_map_gpio[buf[0]].Read();
        packToSend->header = 0xaa;
        packToSend->length = sizeof(package_t) + 1;
        packToSend->cmd = reply | read_gpio;
        bufToSend[0] = val;
        packageAddCrc(packToSend);
        CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 1 + 1);
    }
    return 0;
}
int ProcessSpiPackage(package_t *pack) {
//    write+spi?+datasize2byte+*txdata
//    re|write+status4byte
//
//    read+spi?+datasize2byte
//    re|read+status4byte+*data
//
//    transfer+spi?+datasize2byte+*txdata
//    re|transfer+status4byte+*rxdata
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    uint16_t dataSize2byte = *(uint16_t *) &buf[1];
    int32_t status;
//    static uint8_t dataBuf[1024];
    package_t *packToSend = (package_t *) UserTxBufferFS;
    uint8_t *bufToSend = (uint8_t *) packToSend;
    bufToSend += sizeof(package_t);
    packToSend->header = 0xaa;
    switch (pack->cmd) {
        case write_spi:
            status = HAL_SPI_Transmit(num_map_spi[buf[0]], &buf[3], dataSize2byte, ~0);
            packToSend->cmd = reply | write_spi;
            packToSend->length = sizeof(package_t) + 4;
            *(int32_t *) &bufToSend[0] = status;
            packageAddCrc(packToSend);
            CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 4 + 1);
            break;
        case read_spi:
            status = HAL_SPI_Receive(num_map_spi[buf[0]], &bufToSend[4], dataSize2byte, ~0);
            packToSend->cmd = reply | read_spi;
            packToSend->length = sizeof(package_t) + 4 + dataSize2byte;
            *(int32_t *) &bufToSend[0] = status;
            packageAddCrc(packToSend);
            CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 4 + dataSize2byte + 1);
            break;
        case transfer_spi:
        default:
            status = HAL_SPI_TransmitReceive(num_map_spi[buf[0]], &buf[3], &bufToSend[4], dataSize2byte, ~0);
            packToSend->cmd = reply | transfer_spi;
            packToSend->length = sizeof(package_t) + 4 + dataSize2byte;
            *(int32_t *) &bufToSend[0] = status;
            packageAddCrc(packToSend);
            CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 4 + dataSize2byte + 1);
            break;
    }
    return 0;
}
int ProcessI2cPackage(package_t *pack) {
//    write+iic?+addr+subaddr4byte+subaddrsize+datasize2byte+*data
//    re|write+status4byte
//
//    read+iic?+addr+subaddr4byte+subaddrsize2byte+datasize
//    re|read+status4byte+*data
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    package_t *packToSend = (package_t *) UserTxBufferFS;
    packToSend->header = 0xaa;
    uint8_t *bufToSend = (uint8_t *) packToSend;
    bufToSend += sizeof(package_t);
    int32_t status;
    uint8_t addr = buf[1];
    uint32_t subaddr4byte = *(uint32_t *) &buf[2];
    uint8_t subaddrsize = buf[6];
    uint16_t dataSize2byte = *(uint16_t *) &buf[7];
    if (pack->cmd == write_i2c) {
        status = HAL_I2C_Mem_Write(num_map_i2c[buf[0]], addr, subaddr4byte, subaddrsize, &buf[9], dataSize2byte, ~0);
        packToSend->cmd = reply | write_i2c;
        packToSend->length = sizeof(package_t) + 4;
        *(int32_t *) &bufToSend[0] = status;
        packageAddCrc(packToSend);
        CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 4 + 1);
    } else {
        status = HAL_I2C_Mem_Read(num_map_i2c[buf[0]], addr, subaddr4byte, subaddrsize, &bufToSend[4], dataSize2byte, ~0);
        packToSend->cmd = reply | read_i2c;
        packToSend->length = sizeof(package_t) + 4 + dataSize2byte;
        *(int32_t *) &bufToSend[0] = status;
        packageAddCrc(packToSend);
        CDC_Transmit_FS((uint8_t *) packToSend, sizeof(package_t) + 4 + dataSize2byte + 1);
    }
    return 0;
}


extern "C" int _write(int fd, char *ptr, int len) {
    package_t *packToSend = (package_t *) UserTxBufferFS;
    packToSend->header = 0xaa;
    packToSend->cmd = host_printf;
    packToSend->length = sizeof(package_t) + len;
    uint8_t *bufToSend = (uint8_t *) packToSend;
    bufToSend += sizeof(package_t);
    memcpy(bufToSend, ptr, len);
    packageAddCrc(packToSend);
    while (CDC_Transmit_FS(UserTxBufferFS, sizeof(package_t) + len + 1) != USBD_OK) {}
    return len;
}
