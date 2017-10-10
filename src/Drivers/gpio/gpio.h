#ifndef __DRIVERS_GPIO_H__
#define __DRIVERS_GPIO_H__

#include "stm32f30x_gpio.h"

 
 /**
    \brief Инициализация GPIO на выход
    \param[in] gpio Порт GPIO
    \param[in] pins Битовая маска пинов
    \param[in] outType Режим выхода
                - GPIO_OType_PP       Push Pull
                - PIO_OType_OD        С открытым стоком
    \param[in] pupdType Режим подтяжки
                - GPIO_PuPd_NOPULL    Плавающий
                - GPIO_PuPd_UP        Подтяжка к питанию
                - GPIO_PuPd_DOWN      Подтяжка к земле
    \param[in] speed Скорость
                - GPIO_Speed_Level_1  Fast
                - GPIO_Speed_Level_2  Medium
                - GPIO_Speed_Level_3  High
*/
void GpioInitOutput(GPIO_TypeDef* gpio, uint16_t pins, GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType, GPIOSpeed_TypeDef speed); 


/**
    \brief Инициализация GPIO для альтернативной функции
    \param[in] gpio Порт GPIO
    \param[in] pins Битовая маска пинов
    \param[in] outType Режим выхода
                - GPIO_OType_PP       Push Pull
                - PIO_OType_OD        С открытым стоком
    \param[in] pupdType Режим подтяжки
                - GPIO_PuPd_NOPULL    Плавающий
                - GPIO_PuPd_UP        Подтяжка к питанию
                - GPIO_PuPd_DOWN      Подтяжка к земле
    \param[in] speed Скорость
                - GPIO_Speed_Level_1  Fast
                - GPIO_Speed_Level_2  Medium
                - GPIO_Speed_Level_3  High
*/
void GpioInitAF(GPIO_TypeDef* gpio, uint16_t pins,  GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType, GPIOSpeed_TypeDef speed);


/**
    \brief Инициализация GPIO на вход
    \param[in] gpio Порт GPIO
    \param[in] pins Битовая маска пинов
    \param[in] pupdType Режим подтяжки
                - GPIO_PuPd_NOPULL    Плавающий
                - GPIO_PuPd_UP        Подтяжка к питанию
                - GPIO_PuPd_DOWN      Подтяжка к земле
*/
void GpioInitInput(GPIO_TypeDef* gpio, uint16_t pins, GPIOPuPd_TypeDef pupdType);


/**
    \brief Инициализация GPIO для аналогового входа
    \param[in] gpio Порт GPIO
    \param[in] pins Битовая маска пинов
    \param[in] outType Режим выхода
                - GPIO_OType_PP       Push Pull
                - PIO_OType_OD        С открытым стоком
*/
void GpioInitAnalog(GPIO_TypeDef* gpio, uint16_t pins);

#endif
