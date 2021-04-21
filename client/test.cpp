//
// Created by 17616 on 2021/4/21.
//
#include <windows.h>
#include"client.hpp"
int main(){
    client_t cl;
    cl.AlternateCOM();
    cl.Init("COM4");
    while(1){
        cl.GPIO_Write(0,1);
        Sleep(1000);
        cl.GPIO_Write(0,0);
        Sleep(1000);
    }
    return 0;
}