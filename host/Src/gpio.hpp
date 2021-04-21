//
// Created by 17616 on 2021/4/21.
//

#ifndef HOST_GPIO_HPP
#define HOST_GPIO_HPP
#include "stm32_hal.hpp"

class GPIO {
public:
    GPIO() {}
    GPIO(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin) : gpiox(GPIOx), pin(GPIO_Pin) {}
    void Write(int level) {
        HAL_GPIO_WritePin(gpiox, pin, static_cast<GPIO_PinState>(level));
    }
    int Read(void) {
        return (int) HAL_GPIO_ReadPin(gpiox, pin);
    }
private:
    GPIO_TypeDef *gpiox;
    uint32_t pin;
};
#endif //HOST_GPIO_HPP
