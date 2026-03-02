#ifndef _INT_H_
#define _INT_H_

#include "common.h"

void Initial_prepheral_(void);

void LED_TEST(void);

void HAL_MspInit(void);

#define IWDG_Feed  WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD)

void IWDG_Init_A(void);

void TIM2_INT(void);

void TIM3_INT(void);
	
#endif
