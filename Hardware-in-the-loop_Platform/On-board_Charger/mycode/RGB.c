#include "RGB.h"

void LED_GPIO_CONFIG(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;                         
	
	__HAL_RCC_GPIOB_CLK_ENABLE();			                        
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;   
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;              
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                   
	 
  Red_ON();     Delay_ms(500);                              
}

void Red_ON(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);       
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);         
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);         
}

void Green_ON(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);       
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);         
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);         
}

void Blue_ON(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);       
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);         
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);         
}

