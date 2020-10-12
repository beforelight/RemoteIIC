#include <iostream>
#include"i2c.h"

i2c iic("/dev/i2c-1");

uint8_t val;

int main() {
    iic.Read(0x68, 0x01, &val);
    std::cout << "Hello, World!" << (int)val << std::endl;
    return 0;
}
