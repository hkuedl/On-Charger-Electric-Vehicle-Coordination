

#include "Int.h"

void Initial_prepheral_(void)
{
	HAL_Init();               
	SystemClock_Config_HSE(); 
	Init_TIM_Basic(TIM6);     
	LED_GPIO_CONFIG();        
	OLED_Init();			        
	ADC2_Init();              
	HRTIM_INT();              
}

