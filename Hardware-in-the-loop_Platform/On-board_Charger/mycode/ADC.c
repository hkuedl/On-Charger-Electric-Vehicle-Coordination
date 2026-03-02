

#include "ADC.h"

ADC_HandleTypeDef  ADC2_Handler;      
DMA_HandleTypeDef  ADC2_DMA_Handler;  

__IO uint16_t ADC2_RESULT[3]={0};
__IO uint16_t adc_rest=0x00;

void ADC2_Init(void)
{
	GPIO_InitTypeDef       GPIO_Initure;                                       
	ADC_ChannelConfTypeDef ADC2_ChanConf;                                      
	ADC_MultiModeTypeDef   multimode ;                                         
	
  __HAL_RCC_ADC12_CLK_ENABLE();                                              
  __HAL_RCC_GPIOA_CLK_ENABLE(); 																						 
	__HAL_RCC_DMA1_CLK_ENABLE();                                               
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	
  GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7; 												 
  GPIO_Initure.Mode=GPIO_MODE_ANALOG; 																		   
  GPIO_Initure.Pull=GPIO_NOPULL; 																						 
  HAL_GPIO_Init(GPIOA,&GPIO_Initure);                                        
	
	ADC2_Handler.Instance=ADC2;                                                
	ADC2_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;                 
	ADC2_Handler.Init.Resolution=ADC_RESOLUTION_12B; 													 
	ADC2_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;													 
	ADC2_Handler.Init.ScanConvMode=ENABLE; 															       
	ADC2_Handler.Init.EOCSelection=DISABLE; 																	 
	ADC2_Handler.Init.ContinuousConvMode=ENABLE; 														   
	ADC2_Handler.Init.NbrOfConversion=3; 																       
	ADC2_Handler.Init.DiscontinuousConvMode=DISABLE; 													 
	ADC2_Handler.Init.NbrOfDiscConversion=0; 																	 
	ADC2_Handler.Init.ExternalTrigConv=ADC_EXTERNALTRIG_HRTIM_TRG1;            
	ADC2_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;		 
	ADC2_Handler.Init.DMAContinuousRequests=ENABLE; 													 
	ADC2_Handler.Init.SamplingMode=ADC_SAMPLING_MODE_NORMAL;                   
	ADC2_Handler.Init.GainCompensation = 0;                                    
	ADC2_Handler.Init.LowPowerAutoWait = DISABLE;                              
  ADC2_Handler.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                      
  ADC2_Handler.Init.OversamplingMode = DISABLE;                              
	HAL_ADC_Init(&ADC2_Handler);																							 
	
	multimode.Mode = ADC_MODE_INDEPENDENT;                                     
  HAL_ADCEx_MultiModeConfigChannel(&ADC2_Handler, &multimode);               
	
	ADC2_ChanConf.Channel=ADC_CHANNEL_3;                                       
	ADC2_ChanConf.Rank=ADC_REGULAR_RANK_1;                                     
	ADC2_ChanConf.SamplingTime=ADC_SAMPLETIME_2CYCLES_5;                       
	ADC2_ChanConf.OffsetNumber=ADC_OFFSET_NONE;                                
	ADC2_ChanConf.Offset = 0;                                                  
	ADC2_ChanConf.SingleDiff=ADC_SINGLE_ENDED;                                 
	HAL_ADC_ConfigChannel(&ADC2_Handler,&ADC2_ChanConf);                       
	
	ADC2_ChanConf.Channel=ADC_CHANNEL_2;                                       
	ADC2_ChanConf.Rank=ADC_REGULAR_RANK_2;                                     
	ADC2_ChanConf.SamplingTime=ADC_SAMPLETIME_2CYCLES_5;                       
	ADC2_ChanConf.OffsetNumber=ADC_OFFSET_NONE;                                
	ADC2_ChanConf.Offset = 0;                                                  
	ADC2_ChanConf.SingleDiff=ADC_SINGLE_ENDED;                                 
	HAL_ADC_ConfigChannel(&ADC2_Handler,&ADC2_ChanConf);                       
	
	ADC2_ChanConf.Channel=ADC_CHANNEL_4;                                       
	ADC2_ChanConf.Rank=ADC_REGULAR_RANK_3;                                     
	ADC2_ChanConf.SamplingTime=ADC_SAMPLETIME_2CYCLES_5;                       
	ADC2_ChanConf.OffsetNumber=ADC_OFFSET_NONE;                                
	ADC2_ChanConf.Offset = 0;                                                  
	ADC2_ChanConf.SingleDiff=ADC_SINGLE_ENDED;                                 
	HAL_ADC_ConfigChannel(&ADC2_Handler,&ADC2_ChanConf);                       
	
	HAL_ADCEx_Calibration_Start(&ADC2_Handler, ADC_SINGLE_ENDED);

	ADC2_DMA_Handler.Instance = DMA1_Channel3;                                 
  ADC2_DMA_Handler.Init.Request = DMA_REQUEST_ADC2;                          
  ADC2_DMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;                    
  ADC2_DMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;                        
  ADC2_DMA_Handler.Init.MemInc = DMA_MINC_ENABLE;                            
  ADC2_DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;       
  ADC2_DMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;          
  ADC2_DMA_Handler.Init.Mode = DMA_CIRCULAR;                                 
  ADC2_DMA_Handler.Init.Priority = DMA_PRIORITY_VERY_HIGH ;									 
	HAL_DMA_Init(&ADC2_DMA_Handler);								                           
	__HAL_LINKDMA(&ADC2_Handler,DMA_Handle,ADC2_DMA_Handler);                  
	
	HAL_ADC_Start_DMA(&ADC2_Handler,(uint32_t*)ADC2_RESULT,3);                 
	
	HAL_ADC_Start(&ADC2_Handler);                                              
}
