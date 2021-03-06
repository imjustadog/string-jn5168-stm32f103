#ifndef __PROJECT_H
#define __PROJECT_H

#include "stm32f10x.h"

/*******GPIOA*******/
#define ZIGBEE_PIN_RESET       GPIO_Pin_5
#define STRING_PIN_SWITCH      GPIO_Pin_4

#define STRING_PIN_CEE         GPIO_Pin_8
#define STRING_PIN_CEF         GPIO_Pin_11

#define STRING_PIN_PHZ1        GPIO_Pin_2
#define STRING_PIN_PHZ2        GPIO_Pin_3
#define STRING_PIN_PHZ6        GPIO_Pin_6

#define STRING_PIN_TEMPOP5     GPIO_Pin_0
#define STRING_PIN_TEMPOP6     GPIO_Pin_1

/*******GPIOB*******/
#define STRING_PIN_PHZ3        GPIO_Pin_0
#define STRING_PIN_PHZ4        GPIO_Pin_1
#define STRING_PIN_PHZ5        GPIO_Pin_8

#define ID_PIN_ADR1            GPIO_Pin_4
#define ID_PIN_ADR2            GPIO_Pin_5
#define ID_PIN_ADR3            GPIO_Pin_7
#define ID_PIN_ADR4            GPIO_Pin_6
#define ID_PIN_ADR5            GPIO_Pin_9

/*******GPIOC*******/
#define STRING_PIN_CEA         GPIO_Pin_6
#define STRING_PIN_CEB         GPIO_Pin_7
#define STRING_PIN_CEC         GPIO_Pin_8
#define STRING_PIN_CED         GPIO_Pin_9

#define STRING_PIN_TEMPOP1     GPIO_Pin_0
#define STRING_PIN_TEMPOP2     GPIO_Pin_1
#define STRING_PIN_TEMPOP3     GPIO_Pin_2
#define STRING_PIN_TEMPOP4     GPIO_Pin_3

#define ID_PIN_ADR6            GPIO_Pin_13
#define ID_PIN_ADR7            GPIO_Pin_15
#define ID_PIN_ADR8            GPIO_Pin_14

/*******GPIOD*******/
#define RS485_DE               GPIO_Pin_2              

/*******SPI*******/
#define SPIx                   SPI2
#define SPIx_CLK               RCC_APB1Periph_SPI2
#define SPIx_GPIO              GPIOB
#define SPIx_GPIO_CLK          RCC_APB2Periph_GPIOB 
#define SPIx_PIN_NSS           GPIO_Pin_12
#define SPIx_PIN_SCK           GPIO_Pin_13
#define SPIx_PIN_MISO          GPIO_Pin_14
#define SPIx_PIN_MOSI          GPIO_Pin_15
	
#define FLASH_START_ADDR  0X08020000	
	
#endif
