#include"remote_i2c.h"

int remote_i2c::Read(unsigned char addr, unsigned char reg, unsigned char *val, unsigned int len) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr,
                "Error: Could not set address to 0x%02x: %s\n",
                addr, strerror(errno));
        return -errno;
    }

    while(len--) {
//        if (res >= 0) {
//            res = i2c_smbus_write_byte(fd, reg);
//            reg++;
//            if (res < 0)
//            {
//                fprintf(stderr, "Warning - write failed\n");
//                return -1;
//            }
//        }
        *val = i2c_smbus_read_byte_data(fd,reg);
        reg++;
        val++;
    }
    return 0;
}

int remote_i2c::Write(unsigned char addr, unsigned char reg, const unsigned char *val, unsigned int len) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr,
                "Error: Could not set address to 0x%02x: %s\n",
                addr, strerror(errno));
        return -errno;
    }

    while(len--) {
        i2c_smbus_write_byte_data(fd,reg,*val);
        reg++;
        val++;
    }
    return 0;
}
