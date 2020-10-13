#include <iostream>
#include"remote_i2c.h"
#include"inv_mpu.h"
int remote_i2c_read(void *context,
            unsigned char addr, unsigned char reg,
            unsigned char *val, unsigned int len){
    return static_cast<remote_i2c*>(context)->Read(addr,reg,val,len);
}

int remote_i2c_write(void *context,
             unsigned char addr, unsigned char reg,
             const unsigned char *val, unsigned int len){
    return static_cast<remote_i2c*>(context)->Write(addr,reg,val,len);
}

remote_i2c iic("/dev/i2c-1");
inv::i2c_interface inv_i2c(&iic,remote_i2c_read,remote_i2c_write,
                           remote_i2c_read,remote_i2c_write);
std::shared_ptr<inv::imu> icm;
uint8_t val;

//inv::mpu9250<remote_i2c> imu(iic);


int main() {
	iic.Set(100, 300);
	if(0==inv::Parser(inv_i2c,icm)){
	    icm->init();
        std::cout << icm->report() << std::endl;
        std::cout << "set\r\n" << std::endl;
    }
    printf("helo\r\n");
	return 0;
}

