/*
Реализация задержек при помощи DWT
(Data Watchpoint and Trace Unit)

На основе https://hubstub.ru/stm32/101-funkciya-zaderzhki-stm32.html

Обладает высокой точностью задержек вплоть до нескольких микросекунд.
Примерная систематическая ошибка +0.5us
*/


#ifndef __DELAY_H__
#define __DELAY_H__

#include <stm32f30x.h>

/**
    \brief Необходимые настройки для работы
*/
void DelayInit(void);

/**
    \brief Задержка
    \param[in] us Количество микросекунд задержки
*/

void Delay_us(uint32_t us);




/**
    \bref Задержка, пока с некого события не истечет заданное время
    
    Задержка не константная, а позволяет обернуть некий участок кода
    функциями DelayUntil_start - DelayUntil_delay и он будет выполняться
    заданное время (какое-то время выполняется сам код, оставшееся - 
    DelayUntil_delay
    
    \param[in] us Задержка в микросекундах
    \param[in] t0 Время начала периода, получаемое при помощи DelayUntil_start
*/    
void DelayUntil_delay(uint32_t us, uint32_t t0);

/**
    \brief Получает текущее значение счетчика DWT, чтобы потом использовать в
           DelayUntil_delay
    
    \return Текущее значенеи счетчика
*/
uint32_t DelayUntil_start(void);
#endif
