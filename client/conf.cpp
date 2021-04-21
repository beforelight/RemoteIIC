//
// Created by 17616 on 2021/4/21.
//

#include"../host/Src/conf.h"
#include<mutex>
#include<stdio.h>
#include<iostream>
#include<exception>
extern std::mutex ReceivedPackageMutex;
int CopeReceivedPackage(package_t *pack) {
    if (packageVerify(pack)) {
        if (pack->cmd == host_printf) {
            uint8_t *buf = (uint8_t *) pack;
            buf += sizeof(package_t);
            for (int i = 0; i < pack->length-sizeof(package_t); ++i) {
                putchar(buf[i]);
            }
        }else{
            ReceivedPackageMutex.unlock();
        }
    } else {
        uint8_t *buf = (uint8_t *) pack;
        printf("crc错误 cmd:0x%x length:0x%x crc:0x%x\r\n", pack->cmd, pack->length, buf[pack->length]);
        throw "crc错误\r\n";
    }
    return 0;
}