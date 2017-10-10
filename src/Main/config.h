#ifndef __CONFIG_H__
#define __CONFIG_H__

///////////////////// MOTORS CONTROL CONFIG ////////////////////////////////////

#define MOTORS_GPIO             GPIOE

/*
Управление H-мостом двигаталей

| A | B | направление |
|---|---|-------------|
| 0 | 0 | стоп (gnd)  |
| 0 | 1 | вперед      |
| 1 | 0 | назад       |
| 1 | 1 | стоп (vcc)  |
*/

#define L_MOTOR_A_PIN           GPIO_Pin_10
#define L_MOTOR_B_PIN           GPIO_Pin_11
#define R_MOTOR_A_PIN           GPIO_Pin_8
#define R_MOTOR_B_PIN           GPIO_Pin_9

#define PWM_GPIO                GPIOA
#define L_MOTOR_PWM_PIN         GPIO_Pin_0
#define R_MOTOR_PWM_PIN         GPIO_Pin_1
#define SERVO_PWM_PIN           GPIO_Pin_2
#define L_MOTOR_PINSOURCE       GPIO_PinSource0
#define R_MOTOR_PINSOURCE       GPIO_PinSource1
#define SERVO_PINSOURCE         GPIO_PinSource2

#define SERVO_MIN               500
#define SERVO_MAX               2200
#define MOTOR_MIN               0
#define MOTOR_MAX               20000

/////////////////////////// SONAR CONFIG ///////////////////////////////////////

#define SONAR_TRIG_GPIO			GPIOE
#define SONAR_TRIG_PIN          GPIO_Pin_12
#define SONAR_ECHO_GPIO         GPIOA
#define SONAR_ECHO_PIN          GPIO_Pin_3

////////////////// RASPBERY PI CONNECTION SPI CONFIG ///////////////////////////
#define RASP_SPI                SPI1
#define RASP_SPI_MOSI_PIN       GPIO_Pin_7
#define RASP_SPI_MISO_PIN       GPIO_Pin_6
#define RASP_SPI_SCK_PIN        GPIO_Pin_5
#define RASP_SPI_NSS_PIN        GPIO_Pin_4
#define RASP_SPI_RCC            RCC_APB2Periph_SPI1

//////////////////// USART CONFIG /////////////////////////////////////////////

#define BLUETOOTH_USART                         USART3
#define BLUETOOTH_USART_IRQn                    USART3_IRQn
#define BLUETOOTH_USART_IRQHandler              USART3_IRQHandler

#define BLUETOOTH_USART_RX_GPIO                 GPIOE
#define BLUETOOTH_USART_RX_PIN                  GPIO_Pin_15
#define BLUETOOTH_USART_TX_GPIO                 GPIOB
#define BLUETOOTH_USART_TX_PIN                  GPIO_Pin_10

//9600  for HC-06
//38400 for HC-05
#define BLUETOOTH_USART_BAUD                    9600
#define BLUETOOTH_USART_SEND_BUFFER_SIZE        50

#endif
