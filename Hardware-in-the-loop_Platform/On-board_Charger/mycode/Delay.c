#include "delay.h"

void Delay_ms(__IO uint32_t nCount)
{
  while (nCount != 0)
  {
		nCount--;
	  Delay_us(1000);
  }
}
void Delay_us(__IO uint16_t time)
{
		 TIM6->CNT=0;			
	   while((TIM6->CNT)<time);   
}

void Init_TIM_Basic(TIM_TypeDef *TIM_X)                                                         
{
	TIM_HandleTypeDef TIM_HandleTypeDef_Structure;
	__HAL_RCC_TIM6_CLK_ENABLE();

  TIM_HandleTypeDef_Structure.Instance = TIM_X;
  TIM_HandleTypeDef_Structure.Init.Prescaler = SystemCoreClock/1000000-1;
  TIM_HandleTypeDef_Structure.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_HandleTypeDef_Structure.Init.Period = 0xFFFF; 
  TIM_HandleTypeDef_Structure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_HandleTypeDef_Structure.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&TIM_HandleTypeDef_Structure);
	__HAL_TIM_ENABLE(&TIM_HandleTypeDef_Structure);
}
