

#ifndef _COMMON_H
#define _COMMON_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"
#include "stm32g474xx.h"
#include "clock_config.h"
#include "stdio.h"
#include "delay.h"
#include "Config.h"
#include "Int.h"
#include "ADC.h"
#include "HRTIM.h"
#include "state_machine.h"
#include "RGB.h"
#include "type3.h"
#include "oled.h"

#define CCMRAM  __attribute__((section("ccmram")))
#define Error_Handler()    while(1);

#define Set_485_Receive_data_state  GPIOB->BRR = (uint32_t)GPIO_PIN_5    
#define Set_485_Sent_data_state     GPIOB->BSRR = (uint32_t)GPIO_PIN_5   

#endif 
