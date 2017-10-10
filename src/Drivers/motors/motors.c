#include "motors.h"

#define SERVO_PWM(angle) angle*(SERVO_MAX - SERVO_MIN)/180 + SERVO_MIN

// Настройка моторов и серв
void MotorsInit(void)
{   
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOE,
		ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    
    // Настройка выводов управления мостами
    GpioInitOutput(MOTORS_GPIO, L_MOTOR_A_PIN | 
                          L_MOTOR_B_PIN |
                          R_MOTOR_A_PIN |
                          R_MOTOR_B_PIN,  GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_3);
    
    // Настройка PWM выводов
    GpioInitAF(PWM_GPIO, L_MOTOR_PWM_PIN |
                       R_MOTOR_PWM_PIN |
                       SERVO_PWM_PIN, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_3);
    
    // Настройка таймера
    /*
        SYSCLOCK = 72 MHz
        PWM - 50 Hz
        Tpwm = 1/PWM = 20 ms
        
        F = 72 MHz / 72 = 1000 kHz
        T = 1/F =  1 mks
        Period = Tpwm / T = 20000
        
        Pulse min = 1000
        Pulse max = 2000
		
		1*10^-6 * 10^3
    */
    
    TIM_TimeBaseInitTypeDef motorTimerInit;    
    TIM_TimeBaseStructInit(&motorTimerInit);
    motorTimerInit.TIM_Prescaler = 72;
    motorTimerInit.TIM_Period = 20000;
    TIM_TimeBaseInit(TIM2, &motorTimerInit);
    
    
    // Настройка каналов таймера
    TIM_OCInitTypeDef channelInit;    
    TIM_OCStructInit(&channelInit);    
    channelInit.TIM_OCMode = TIM_OCMode_PWM1;
    channelInit.TIM_OutputState = TIM_OutputState_Enable;
    channelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // L motor
    channelInit.TIM_Pulse = 0;
    TIM_OC1Init(TIM2, &channelInit);    
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    GPIO_PinAFConfig(PWM_GPIO, L_MOTOR_PINSOURCE, GPIO_AF_1);    
    
    // R motor
    channelInit.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &channelInit);    
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    GPIO_PinAFConfig(PWM_GPIO, R_MOTOR_PINSOURCE, GPIO_AF_1);
    
    // Servo
    channelInit.TIM_Pulse = SERVO_PWM(90);
    TIM_OC3Init(TIM2, &channelInit);    
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    GPIO_PinAFConfig(PWM_GPIO, SERVO_PINSOURCE, GPIO_AF_1);    
    
    TIM_Cmd(TIM2, ENABLE);
	
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);   
    //NVIC_EnableIRQ(TIM2_IRQn);
}

/*void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)!=RESET)
	{
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        GPIOE->ODR ^= L_MOTOR_B_PIN;
    }
}*/

/**
    \brief Устанавливает угол сервомашинки
    \param[in] angle Угол поворота в градусах [0; 180]
*/
void SetServo(uint8_t angle)
{   
    uint16_t pulse = SERVO_PWM(angle);
    TIM2->CCR3 = pulse;
}

// Управляет ключами H-моста левого мотора
void L_Motor_SetDirection(bool a, bool b)
{
    if(a) MOTORS_GPIO->ODR |= L_MOTOR_A_PIN;
    else MOTORS_GPIO->ODR &= ~L_MOTOR_A_PIN;

    if(b) MOTORS_GPIO->ODR |= L_MOTOR_B_PIN;
    else MOTORS_GPIO->ODR &= ~L_MOTOR_B_PIN;
}

// Управляет ключами H-моста правого мотора
void R_Motor_SetDirection(bool a, bool b)
{
    if(a) MOTORS_GPIO->ODR |= R_MOTOR_A_PIN;
    else MOTORS_GPIO->ODR &= ~R_MOTOR_A_PIN;

    if(b) MOTORS_GPIO->ODR |= R_MOTOR_B_PIN;
    else MOTORS_GPIO->ODR &= ~R_MOTOR_B_PIN;
}

// Устанавлвиает скважность PWM левого мотора
void L_Motor_SetPWM(uint8_t pwm)
{
    TIM2->CCR1 = pwm * (MOTOR_MAX - MOTOR_MIN)/255 + MOTOR_MIN;
}

// Устанавлвиает скважность PWM левого мотора
void R_Motor_SetPWM(uint8_t pwm)
{
    TIM2->CCR2 = pwm * (MOTOR_MAX - MOTOR_MIN)/255 + MOTOR_MIN;
}

// Управляет скоростью и направлением вращения моторов
void MotorsControl(int16_t left, int16_t right)
{
    if(left > 0)
    {
        L_Motor_SetDirection(false, true);
        L_Motor_SetPWM(left);
    }
    else if(left < 0)
    {
        L_Motor_SetDirection(true, false);
        L_Motor_SetPWM(-left);
    }
    else
    {
        L_Motor_SetDirection(false, false);
        L_Motor_SetPWM(0);
    }
    
    if(right > 0)
    {
        R_Motor_SetDirection(false, true);
        R_Motor_SetPWM(right);
    }
    else if(right < 0)
    {
        R_Motor_SetDirection(true, false);
        R_Motor_SetPWM(-right);
    }
    else
    {
        R_Motor_SetDirection(false, false);
        R_Motor_SetPWM(0);
    }
}
