//
// Created by 17616 on 2020/10/12.
//

#ifndef REMOTEIIC_I2C_H
#define REMOTEIIC_I2C_H

#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

#define I2C_DEFAULT_TIMEOUT     1
#define I2C_DEFAULT_RETRY       3

class i2c {
public:
    i2c(const char *i2c_file_name = "/dev/i2c-1") : isOpen(0) {
        if ((file = open(i2c_file_name, O_RDWR)) < 0) {
            printf("Unable to open i2c control file %s\r\n", i2c_file_name);
        } else {
            isOpen = 1;
            Set();
        }
    }


    int IsOpen() { return isOpen; }

    int Set(unsigned int timeout = I2C_DEFAULT_TIMEOUT, unsigned int retry = I2C_DEFAULT_RETRY) {
        if (file == 0)
            return -1;
        if (ioctl(file, I2C_TIMEOUT, timeout ? timeout : I2C_DEFAULT_TIMEOUT) < 0)
            return -1;
        if (ioctl(file, I2C_RETRIES, retry ? retry : I2C_DEFAULT_RETRY) < 0)
            return -1;
        return 0;
    }

    int Read(unsigned char addr,
             unsigned char reg,
             unsigned char *val,
             unsigned int len = 1);

    int Write(unsigned char addr,
              unsigned char reg,
              const unsigned char *val,
              unsigned int len = 1);

private:
    int file;
    int isOpen;
};


#endif //REMOTEIIC_I2C_H
