#include "sonar.h"

#define PERIOD 0xffff                           // Макс. счет таймера
#define MAX_DIST 400

uint16_t _startTime;                            // Время начала импульса Echo
bool _sonarState;                               // Состояние
bool _isMeasured;                               // Было ли сделано измерение
uint16_t _lastDist;                             // Последнее измеренное расстояние
DistanceMeasuredHandler _distMeasuredHandler;   // Обработчик

// Расчитывает длину импульса по времени начал и конца
uint32_t GetDuration(uint16_t start, uint16_t end);

void SonarInit(DistanceMeasuredHandler handler)
{
    _distMeasuredHandler = handler;
    
	RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOE,
		ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
    
    GpioInitOutput(SONAR_TRIG_GPIO, SONAR_TRIG_PIN,  GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_3);
    GpioInitAF(SONAR_ECHO_GPIO, SONAR_ECHO_PIN,  GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_Level_3);
    GPIO_PinAFConfig(SONAR_ECHO_GPIO, GPIO_PinSource3, GPIO_AF_9);  
	
	/*
	SYSCLOCK = 72 MHz
	Длительность сигнала Echo [150us; 25ms], 38ms для +inf
    
    T = 1/F
    F = SYSCLOCK/PRESCALER => T = PRESCALER/SYSCLOCK
    T = 1us
    PRESCALER = 10us * 72 MHz = 10*10^-6 * 72 * 10^6 = 720
    
    150 000  65535
	*/
	
	TIM_TimeBaseInitTypeDef timerInit;    
    TIM_TimeBaseStructInit(&timerInit);
    timerInit.TIM_Prescaler = 72 * 4;
    timerInit.TIM_Period = PERIOD;
    TIM_TimeBaseInit(TIM15, &timerInit);
    
    TIM_ICInitTypeDef captureInit;
    TIM_ICStructInit(&captureInit);
    captureInit.TIM_Channel = TIM_Channel_2; 
    captureInit.TIM_ICFilter = 0;
    captureInit.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    captureInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    captureInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM15, &captureInit);
        
    _startTime = 0;
    _sonarState = false;
    _isMeasured = false;
    
    TIM_Cmd(TIM15, ENABLE);
	
	//TIM_ITConfig(TIM15, TIM_IT_Update, ENABLE);  
    TIM_ITConfig(TIM15, TIM_IT_CC2, ENABLE);
    NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    
}


void SonarStartMeasure(void)
{
    SONAR_TRIG_GPIO->ODR |= SONAR_TRIG_PIN;
    Delay_us(10);
    SONAR_TRIG_GPIO->ODR &= ~SONAR_TRIG_PIN;
}


uint16_t SonarMeasureDistance(void)
{
    uint32_t dist = 0;
    const uint8_t times = 1;
   
    for(uint8_t i = 0; i<times; i++)
    {
        _isMeasured = false;
        SonarStartMeasure();
        while(!_isMeasured);
        
        if(_lastDist >= MAX_DIST)
            return _lastDist;
        
        dist+=_lastDist;
    }
    
    return dist / times;
   
}

////////////////////////////////////////////////////////////////////////////////

void TIM1_BRK_TIM15_IRQHandler()
{
    if(TIM_GetITStatus(TIM15, TIM_IT_CC2)!=RESET)
    {
        TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);
        
        if(!_sonarState)
        {
            _startTime = TIM15->CCR2;
            _sonarState = true;
        }
        else
        {
            _sonarState = false;
            _isMeasured = true;
            _lastDist = GetDuration(_startTime, TIM15->CCR2) / 58.0f;
            _distMeasuredHandler(_lastDist);
        }
    }
}

// Расчитывает длину импульса по времени начал и конца
uint32_t GetDuration(uint16_t start, uint16_t end)
{
    //Таймер может переполниться
    if(end > start)
        return (end - start)*4;
    else
        return (PERIOD - start + end)*4;
            
}


