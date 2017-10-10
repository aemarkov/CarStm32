/*
Примем команд ручного управления по UART
Передача произвольных данных
*/

#ifndef __UART_H__
#define __UART_H__

#include <config.h>
#include <stm32f30x_usart.h>
#include <Drivers/gpio/gpio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// Тип коллбека приема команд
// скорость левого и скорость правого двигателей
typedef void (*CommandHandler)(int16_t, int16_t);

/**
    \brief Настраивает UART
    \param[in] baud Скорость обмена (бит/с)
    \param[in] handler Указатель на функцию-обработчик принятых команд
*/
void UartInit(int baud, CommandHandler handler);

/**
    \brief Отправка произвольных данных
    \param[in] data Указатель на буффер
    \param[in] length Размер данных 
*/
void UartSend(uint8_t* data, uint8_t length);

#endif
