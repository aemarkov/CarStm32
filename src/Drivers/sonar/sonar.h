/*
    Измерение расстояния сонаром.
    Для измерения времени задержки используется
    таймер (режим захвата)
*/

#ifndef __SONAR_H__
#define __SONAR_H__

#include <stdint.h>
#include <stdbool.h>

#include <stm32f30x_rcc.h>
#include <stm32f30x_tim.h>
#include <Drivers/gpio/gpio.h>
#include <Drivers/delay/delay.h>

#include <config.h>

/**
    \brief Тип коллбека завершенности измерения
    \param[in] Dist расстояние в сантиметрах
*/
typedef void (*DistanceMeasuredHandler)(uint16_t dist);

/**
    \brief Инициализация
    \param[in] hander Указатель на функцию-обработчик завершенности измерения
*/
void SonarInit(DistanceMeasuredHandler handler);

/**
    \brief Посылает импульс начала измерения
    
    Эта функция только запускает измерение. Результат будет доступен через
    некоторое время с вызовом обработчика
*/
void SonarStartMeasure(void);

/**
    \brief Блокирующее измерение расстояния
    Функция не вернет управление, пока результат не будет получен

    \return Расстояние в сантиметрах
*/
uint16_t SonarMeasureDistance(void);
#endif
