//
// Created by 17616 on 2020/10/12.
//

#ifndef REMOTEIIC_REMOTE_I2C_H
#define REMOTEIIC_REMOTE_I2C_H

#include <cstdio>
#include <cstring>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "smbus.h"
//#include <i2c/smbus.h>
//#include "i2cbusses.h"
//#include "util.h"

class remote_i2c {
public:
    remote_i2c(const char *_bus = "/dev/i2c-1") : isOpen(0),bus(_bus) {
        if ((fd = open(bus,O_RDWR)) < 0) {
            printf("Unable to open remote_i2c control file %s\r\n", bus);
        } else {
            isOpen=true;
        }
    }

    ~remote_i2c(){
        if(isOpen){
            close(fd);
        }
    }


    int is_open() { return isOpen; }

    int Read(unsigned char addr,
             unsigned char reg,
             unsigned char *val,
             unsigned int len = 1);

    int Write(unsigned char addr,
              unsigned char reg,
              const unsigned char *val,
              unsigned int len = 1);

private:
    const char* bus;
    int fd;
    int isOpen;
};


#endif //REMOTEIIC_REMOTE_I2C_H
