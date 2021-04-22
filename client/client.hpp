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
#include<mutex>
#include "SerialPort.h"

#include<cstdio>
static std::mutex ptf_mux;
#define INV_NO_DEBUG
#ifndef INV_NO_DEBUG
#define INV_DEBUG_(fmt, ...) \
    printf("%s:%d:debug: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) ptf_mux.lock();INV_DEBUG_(__VA_ARGS__, "");fflush(stdout);ptf_mux.unlock()
#else
#define INV_DEBUG(...)
#endif //INV_NO_DEBUG

using std::string;
using std::exception;
using std::cout;
using std::cin;
using std::endl;
using std::vector;
class client_t {
public:
    client_t() {}
    int Init(int port,unsigned long baud = 115200);
    void AlternateCOM();
    int ServerBackgrand(char* _buf,UINT _len);
//    int CheckRecive();

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
    CSerialPort srl;
    std::thread td;
};


#endif //CLIENT_CLIENT_HPP
