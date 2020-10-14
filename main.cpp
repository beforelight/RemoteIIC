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

float acc[3];

int main() {
	if(0==inv::Parser(inv_i2c,icm)){
	    icm->init();
	    usleep(100000);
	    icm->read_sensor_blocking();
	    icm->converter(acc,acc+1,acc+2,NULL,NULL,NULL);
        printf(icm->report().c_str()) ;
        printf("\r\naccel %.3f %.3f %.3f \r\n",acc[0],acc[1],acc[2]);
    }
    printf("helo\r\n");
	return 0;
}

