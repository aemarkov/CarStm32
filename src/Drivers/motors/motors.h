/*
    Управление моторами и сервомашинкой
*/

#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

#include <stm32f30x_rcc.h>
#include <stm32f30x_tim.h>
#include <Drivers/gpio/gpio.h>

#include <config.h>


/**
    \brief Настройка моторов и серв
*/
void MotorsInit(void);

/**
    \brief Устанавливает угол сервомашинки
    \param[in] angle Угол поворота в градусах [0; 180]
*/
void SetServo(uint8_t angle);

/**
    \brief Управляет ключами H-моста левого мотора

    | A | B | направление |
    |---|---|-------------|
    | 0 | 0 | стоп (gnd)  |
    | 0 | 1 | вперед      |
    | 1 | 0 | назад       |
    | 1 | 1 | стоп (vcc)  |
*/
void L_Motor_SetDirection(bool a, bool b);

/**
    \brief Управляет ключами H-моста правого мотора

    | A | B | направление |
    |---|---|-------------|
    | 0 | 0 | стоп (gnd)  |
    | 0 | 1 | вперед      |
    | 1 | 0 | назад       |
    | 1 | 1 | стоп (vcc)  |
*/
void R_Motor_SetDirection(bool a, bool b);

/**
    \brief Устанавлвиает скважность PWM левого мотора
    \param[in] pwm Скважность [0; 255]
*/
void L_Motor_SetPWM(uint8_t pwm);

/**
    \brief Устанавлвиает скважность PWM левого мотора
    \param[in] pwm Скважность [0; 255]
*/
void R_Motor_SetPWM(uint8_t pwm);

/**
    \brief Управляет скоростью и направлением вращения моторов
    <0 - назад
     0 - GND-стоп
    >0 - вперед

    \param[in] left Скорость левого двигателя [-255; 255]
    \param[in] right Скорость правого двигателя [-255; 255]
*/
void MotorsControl(int16_t left, int16_t right);

#endif
