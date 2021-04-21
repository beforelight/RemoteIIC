//
// Created by 17616 on 2021/4/21.
//

#ifndef CLIENT_CLIENT_HPP
#define CLIENT_CLIENT_HPP
#include<string>
#include<iostream>
#include<vector>
#include<memory>
#include<thread>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::endl;
using std::vector;
class client_t {
public:
    client_t();
    int Init(string port,unsigned long baud = 115200);
    void AlternateCOM();
    static void ServerBackgrand(client_t* clt);

    void GPIO_Write(int gpio,uint8_t val);
    uint8_t GPIO_Read(int gpio);

    typedef enum __inv_i2c_direction {
        inv_i2c_direction_Write = 0U, /*!< Master transmit. */
        inv_i2c_direction_Read = 1U  /*!< Master receive. */
    } inv_i2c_direction_t;

    typedef struct __inv_i2c_transfer {
        uint8_t slaveAddress;      /*!< 7-bit slave address. */
        uint8_t subAddressSize;     /*!< A size of the command buffer. */
        uint32_t subAddress;        /*!< A sub address. Transferred MSB first. */
        void *volatile data;        /*!< A transfer buffer. */
        volatile uint32_t dataSize;          /*!< A transfer size. */
        inv_i2c_direction_t direction; /*!< A transfer direction, read or write. */
        /*******************************************************/
        //you like
        uint8_t i2cx;
    } inv_i2c_transfer_t;

    typedef struct __inv_spi_transfer {
        uint8_t *volatile txData;          /*!< Send buffer. */
        uint8_t *volatile rxData;          /*!< Receive buffer. */
        volatile uint32_t dataSize; /*!< Transfer bytes. */
        /*******************************************************/
        //you like
        uint8_t spix;
    } inv_spi_transfer_t;

    int32_t SPI_Transfer(inv_spi_transfer_t& tf);
    int32_t I2C_Transfer(inv_i2c_transfer_t& tf);

private:
    std::shared_ptr<serial::Serial> srl;
    std::thread td;
};


#endif //CLIENT_CLIENT_HPP
