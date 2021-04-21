//
// Created by 17616 on 2021/4/21.
//

#include "client.hpp"
#include <assert.h>
#include<mutex>
#include"../host/Src/conf.h"
#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif
std::mutex ReceivedPackageMutex;
uint8_t cmdBuffer[1024];
uint8_t txBuf[2048];
client_t::client_t() {
}
//交互，初始化串口
int client_t::Init(string port, unsigned long baud) {
    cout << "port = " << port << " baud = " << baud << endl;
    srl.reset(new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000 * 10)));
    cout << "Is the serial port open?";
    if (srl->isOpen()) {
        td = std::thread(ServerBackgrand, this);
        cout << " Yes." << endl;

    } else {
        cout << " No." << endl;
    }
    return 0;
}
void client_t::AlternateCOM() {
    vector<serial::PortInfo> devices_found = serial::list_ports();
    for (int i = 0; i < devices_found.size(); ++i) {
        printf("%d: %s, %s, %s\r\n",
               i,
               devices_found[i].port.c_str(),
               devices_found[i].description.c_str(),
               devices_found[i].hardware_id.c_str());
    }
}
void client_t::ServerBackgrand(client_t *clt) {
    ReceivedPackageMutex.lock();
    static std::vector<uint8_t> buffer;
    std::shared_ptr<serial::Serial> srl = clt->srl;
    while (1) {
        while (srl->available()) {
            srl->read(buffer, srl->available());
            uint8_t *UserRxBufferFS = &buffer[0];
            uint32_t LenLen = buffer.size();
            uint32_t *Len = &LenLen;


            /* USER CODE BEGIN 6 */

            //在原地处理
            static int8_t onPackage = 0;
            uint32_t idx = 0;//处理到当前缓存的第几个
            static uint32_t lastCopyLen = 0;//上一次拷贝了多少长度的数据
            package_t *package;
            while ((*Len) - idx) {
                if (onPackage == 0) {//检查是否接收到头
                    for (uint32_t i = idx; i < (*Len); ++i) {
                        if (UserRxBufferFS[i] == 0xaa) {//检查出帧头后开始一部分拷贝
                            onPackage = 1;
                            package = (package_t *) &UserRxBufferFS[i];
                            if ((*Len) - idx < sizeof(package_t) || package->length + 1 > (*Len) - idx) {
                                lastCopyLen = (*Len) - idx;
                                memcpy(cmdBuffer, package, lastCopyLen);
                                idx = (*Len);
                            } else {
                                idx += package->length + 1;
                                memcpy(cmdBuffer, package, package->length + 1);//拷贝完一个指令，
                                //执行指令
                                CopeReceivedPackage(package);
                                onPackage = 0;
                            }
                            break;
                        } else {
                            idx++;
                        }
                    }
                } else {
                    package = (package_t *) &cmdBuffer[0];
                    if (lastCopyLen < sizeof(package_t)) {//保证拷贝了长度信息
                        idx = MIN(sizeof(package_t) - lastCopyLen, (*Len));
                        memcpy(&cmdBuffer[lastCopyLen], &UserRxBufferFS[0], idx);
                        lastCopyLen += idx;
                    } else {
                        if (package->length + 1 > (*Len) - idx + lastCopyLen) {//接收的字符还不够
                            memcpy(&cmdBuffer[lastCopyLen], &UserRxBufferFS[idx], (*Len) - idx);
                            idx = (*Len);
                            lastCopyLen += (*Len) - idx;
                        } else {
                            memcpy(&cmdBuffer[lastCopyLen], &UserRxBufferFS[idx], package->length + 1 - lastCopyLen);
                            idx += package->length + 1 - lastCopyLen;//拷贝完一个指令，
                            //执行指令
                            CopeReceivedPackage(package);
                            onPackage = 0;
                        }
                    }
                }
            }

        }
    }
}
void client_t::GPIO_Write(int gpio, uint8_t val) {
    package_t *pack = (package_t *) txBuf;
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    pack->header = 0xaa;
    pack->length = sizeof(package_t) + 2;
    pack->cmd = write_gpio;
    buf[0] = gpio;
    buf[1] = val;
    packageAddCrc(pack);
    assert(sizeof(package_t) + 2 + 1 == srl->write(txBuf, sizeof(package_t) + 2 + 1));
}
uint8_t client_t::GPIO_Read(int gpio) {
    package_t *pack = (package_t *) txBuf;
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    uint8_t val;
    pack->header = 0xaa;
    pack->length = sizeof(package_t) + 1;
    pack->cmd = read_gpio;
    buf[0] = gpio;
    packageAddCrc(pack);
    assert(sizeof(package_t) + 1 + 1 == srl->write(txBuf, sizeof(package_t) + 1 + 1));
    ReceivedPackageMutex.lock();

    pack = (package_t *) cmdBuffer;
    buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    assert(pack->cmd == reply | read_gpio);
    val = buf[0];
    return val;
}
int32_t client_t::SPI_Transfer(client_t::inv_spi_transfer_t &tf) {
    package_t *pack = (package_t *) txBuf;
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    pack->header = 0xaa;

    package_t *packReceived = (package_t *) cmdBuffer;
    uint8_t *bufReceived = (uint8_t *) packReceived;
    bufReceived += sizeof(package_t);

    int32_t status;
//    write+spi?+datasize2byte+*data
//    re|write+status4byte
//
//    read+spi?+datasize2byte
//    re|read+status4byte+*data
//
//    transfer+spi?+datasize2byte+*txdata
//    re|transfer+status4byte+*rxdata

    if (tf.rxData == NULL) {
        pack->length = sizeof(package_t) + 1 + 2 + tf.dataSize;
        pack->cmd = write_spi;
        buf[0] = tf.spix;
        *(uint16_t *) &buf[1] = tf.dataSize;
        memcpy(&buf[3], tf.txData, tf.dataSize);
        packageAddCrc(pack);
        assert(sizeof(package_t) + 1 + 2 + tf.dataSize + 1 == srl->write(txBuf, sizeof(package_t) + 1 + 2 + tf.dataSize + 1));
        ReceivedPackageMutex.lock();
        assert(packReceived->cmd == (reply | write_spi));
        status = *(int32_t *) &bufReceived[0];
        return status;
    } else if (tf.txData == NULL) {
        pack->length = sizeof(package_t) + 1 + 2;
        pack->cmd = read_spi;
        buf[0] = tf.spix;
        *(uint16_t *) &buf[1] = tf.dataSize;
        packageAddCrc(pack);
        assert(sizeof(package_t) + 1 + 2 + 1 == srl->write(txBuf, sizeof(package_t) + 1 + 2 + 1));
        ReceivedPackageMutex.lock();
        assert(packReceived->cmd == (reply | read_spi));
        status = *(int32_t *) &bufReceived[0];
        memcpy(tf.rxData, &bufReceived[4], tf.dataSize);
        return status;
    } else {
        pack->length = sizeof(package_t) + 1 + 2 + tf.dataSize;
        pack->cmd = transfer_spi;
        buf[0] = tf.spix;
        *(uint16_t *) &buf[1] = tf.dataSize;
        memcpy(&buf[3], tf.txData, tf.dataSize);
        packageAddCrc(pack);
        assert(sizeof(package_t) + 1 + 2 + tf.dataSize + 1 == srl->write(txBuf, sizeof(package_t) + 1 + 2 + tf.dataSize + 1));
        ReceivedPackageMutex.lock();
        assert(packReceived->cmd == (reply | transfer_spi));
        status = *(int32_t *) &bufReceived[0];
        memcpy(tf.rxData, &bufReceived[4], tf.dataSize);
        return status;
    }
    return 0;
}
int32_t client_t::I2C_Transfer(client_t::inv_i2c_transfer_t &tf) {
//    write+iic?+addr+subaddr4byte+subaddrsize+datasize2byte+*data
//    re|write+status4byte
//
//    read+iic?+addr+subaddr4byte+subaddrsize2byte+datasize
//    re|read+status4byte+*data
    package_t *pack = (package_t *) txBuf;
    uint8_t *buf = (uint8_t *) pack;
    buf += sizeof(package_t);
    pack->header = 0xaa;

    package_t *packReceived = (package_t *) cmdBuffer;
    uint8_t *bufReceived = (uint8_t *) packReceived;
    bufReceived += sizeof(package_t);

    int32_t status;
    buf[0] = tf.slaveAddress;
    *(uint32_t *) &buf[1] = tf.subAddress;
    buf[5] = tf.subAddressSize;
    *(uint16_t *) &buf[6] = tf.dataSize;

    if (tf.direction == inv_i2c_direction_Write) {
        pack->cmd = write_i2c;
        pack->length = sizeof(package_t) + 1 + 4 + 1 + 2 + tf.dataSize;
        memcpy(&buf[8], tf.data, tf.dataSize);
        packageAddCrc(pack);
        assert(sizeof(package_t) + 1 + 4 + 1 + 2 + tf.dataSize + 1
               == srl->write(txBuf, sizeof(package_t) + 1 + 4 + 1 + 2 + tf.dataSize + 1));
        ReceivedPackageMutex.lock();
        assert(packReceived->cmd == (reply | write_i2c));
        status = *(int32_t *) &bufReceived[0];
        return status;
    } else {
        pack->cmd = read_i2c;
        pack->length = sizeof(package_t) + 1 + 4 + 1 + 2;
        packageAddCrc(pack);
        assert(sizeof(package_t) + 1 + 4 + 1 + 2 + 1
               == srl->write(txBuf, sizeof(package_t) + 1 + 4 + 1 + 2 + 1));
        ReceivedPackageMutex.lock();
        assert(packReceived->cmd == (reply | read_i2c));
        status = *(int32_t *) &bufReceived[0];
        memcpy(tf.data, &bufReceived[4], tf.dataSize);
        return status;
    }
    return 0;
}
