#include "gpio.h"


void GpioInitOutput(GPIO_TypeDef* gpio, uint16_t pins, GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType, GPIOSpeed_TypeDef speed)
{
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_structure.GPIO_OType = outType;
    gpio_init_structure.GPIO_Pin = pins;
    gpio_init_structure.GPIO_Speed = speed;
    gpio_init_structure.GPIO_PuPd = pupdType;
    GPIO_Init(gpio,&gpio_init_structure);
}

void GpioInitAF(GPIO_TypeDef* gpio, uint16_t pins,  GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType, GPIOSpeed_TypeDef speed)
{
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_structure.GPIO_OType = outType;
    gpio_init_structure.GPIO_Pin = pins;
    gpio_init_structure.GPIO_Speed = speed;
    gpio_init_structure.GPIO_PuPd = pupdType;
    GPIO_Init(gpio,&gpio_init_structure);
}
    
void GpioInitInput(GPIO_TypeDef* gpio, uint16_t pins, GPIOPuPd_TypeDef pupdType)
{
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_structure.GPIO_Pin = pins;    
    gpio_init_structure.GPIO_PuPd = pupdType;
    GPIO_Init(gpio,&gpio_init_structure);
}

void GpioInitAnalog(GPIO_TypeDef* gpio, uint16_t pins)
{
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_structure.GPIO_Pin = pins;    
    GPIO_Init(gpio,&gpio_init_structure);
}
