//
// Created by 17616 on 2020/10/12.
//

#include "i2c.h"

int i2c::Read(unsigned char addr, unsigned char reg, unsigned char *val, unsigned int len) {
    if (isOpen == 0) { return 1; }
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    messages[0].addr = addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(reg);
    messages[0].buf = &reg;

    messages[1].addr = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len = len;
    messages[1].buf = val;

    packets.msgs = messages;
    packets.nmsgs = 2;
    if (ioctl(file, I2C_RDWR, &packets) < 0) {
        printf("Unable to send data\r\n");
        return 1;
    }
    return 0;

}

int i2c::Write(unsigned char addr, unsigned char reg, const unsigned char *val, unsigned int len) {
    if (isOpen == 0) { return 1; }
    int rtv = 0;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    messages[0].addr = addr;
    messages[0].flags = 0;
    messages[0].len = len + 1;
    messages->buf = (unsigned char *) malloc(len + 1);
    messages->buf[0]=reg;
    memcpy(&messages->buf[1],val,len);
    packets.msgs = messages;
    packets.nmsgs = 1;
    if (ioctl(file, I2C_RDWR, &packets) < 0) {
        printf("Unable to send data\r\n");
        rtv = 1;
    }
    free(messages->buf);
    return rtv;
}
