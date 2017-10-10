#include "stm32f30x.h"
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>

#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include <Drivers/gpio/gpio.h>
#include <Drivers/gpio/discovery_leds.h>
#include <Drivers/motors/motors.h>
#include <Drivers/sonar/sonar.h>
#include <Drivers/delay/delay.h>


void sonarHandler(uint16_t dist)
{
}


int main(void)
{   
    DelayInit();
    MotorsInit();
    SonarInit(sonarHandler);
    
    while(1)
    {         
    }
}