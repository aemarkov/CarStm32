#include "delay.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void DelayInit(void)
{
    //разрешаем использовать счётчик
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    //обнуляем значение счётного регистра
	DWT_CYCCNT  = 0;
    //запускаем счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk; 
}

void Delay_us(uint32_t us)
{
    uint32_t t0 =  DWT->CYCCNT;
    uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
    while ((DWT->CYCCNT - t0) < us_count_tic) ;
}

uint32_t DelayUntil_start(void)
{
    return DWT->CYCCNT;
}

void DelayUntil_delay(uint32_t us, uint32_t t0)
{
    uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
    while ((DWT->CYCCNT - t0) < us_count_tic) ;
}


/*
Работает даже при переполнении
(заменим uint32_t на uint8_t для
простоты)

1. CYCCNT = 50
   t0 = CYCCNT = 50
   delay = 50
2. ...
3. CYCCNT = 101
   t0 = 50
   CYCCNT - t0 = 51 > delay


1. CYCCNT = 250
   t0 = CYCCNT = 250
   delay = 50
2. ...
3. CYCCNT = 45
   CYCCNT - t0 = 51 > delay
*/
