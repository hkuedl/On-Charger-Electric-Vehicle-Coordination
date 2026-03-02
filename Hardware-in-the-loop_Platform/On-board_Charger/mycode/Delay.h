#ifndef __DELAY_H__
#define __DELAY_H__

#include "common.h"

void Delay_us(uint16_t time);
void Delay_ms(__IO uint32_t nCount);

void Init_TIM_Basic(TIM_TypeDef *TIM_X);  
#endif
