

#include "HRTIM.h"

HRTIM_HandleTypeDef  HRTIM1_structure;

void HRTIM_INT(void)
{
	GPIO_InitTypeDef           GPIO_InitStruct;                           
	HRTIM_TimeBaseCfgTypeDef   pTimeBaseCfg  = {0};                       
  HRTIM_TimerCfgTypeDef      pTimerCfg     = {0};                       
  HRTIM_TimerCtlTypeDef      pTimerCtl     = {0};                       
  HRTIM_OutputCfgTypeDef     pOutputCfg    = {0};                       
	HRTIM_CompareCfgTypeDef    pCompareCfg   = {0};                       
	HRTIM_DeadTimeCfgTypeDef   pDeadTimeCfg  = {0};                       
	HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg= {0};                       
	
	__HAL_RCC_HRTIM1_CLK_ENABLE();                                        
	__HAL_RCC_GPIOA_CLK_ENABLE();                                         
	
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;                          
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                               
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;                                 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    
	GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;                         
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                               
	
  HRTIM1_structure.Instance = HRTIM1;                                   
  HRTIM1_structure.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;        
  HRTIM1_structure.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;            
  HAL_HRTIM_Init(&HRTIM1_structure);                                    

  pTimeBaseCfg.Period = PWM_PERIOD;  
  pTimeBaseCfg.RepetitionCounter = 0x00;                                
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;             
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;                            
  HAL_HRTIM_TimeBaseConfig(&HRTIM1_structure, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg); 
	HAL_HRTIM_TimeBaseConfig(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg);
  
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;                   
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;                        
  pTimerCfg.DMASrcAddress = 0x0000;                                     
  pTimerCfg.DMADstAddress = 0x0000;                                     
  pTimerCfg.DMASize = 0x1;                                              
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;                   
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;          
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_ENABLED;                      
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_ENABLED;                      
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;                            
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;                      
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;              
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;             
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;       
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;       
	pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;              
 
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;               
  HAL_HRTIM_WaveformTimerConfig(&HRTIM1_structure, HRTIM_TIMERINDEX_MASTER, &pTimerCfg);
  
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;                      
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;                           
  pTimerCfg.DMASrcAddress = 0x0000;                                     
  pTimerCfg.DMADstAddress = 0x0000;                                     
  pTimerCfg.DMASize = 0x1;                                              
	pTimerCfg.DACSynchro=HRTIM_DACSYNC_DACTRIGOUT_1;                      
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;                  
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;                    
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;                   
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;     
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;              
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;            
  HAL_HRTIM_WaveformTimerConfig(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg);
	
	pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;		                  
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;           
  HAL_HRTIM_WaveformTimerControl(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl);

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;                      
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;                        
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1; 									
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;                      
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;                
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;                  
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;      
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	
  HAL_HRTIM_WaveformOutputConfig(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg);

	pCompareCfg.AutoDelayedMode=HRTIM_AUTODELAYEDMODE_REGULAR;             
	pCompareCfg.CompareValue=5000;                                         
	HAL_HRTIM_WaveformCompareConfig(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_COMPAREUNIT_1,&pCompareCfg);

	pDeadTimeCfg.FallingLock=HRTIM_TIMDEADTIME_FALLINGLOCK_READONLY;        
	pDeadTimeCfg.FallingSign=HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;        
	pDeadTimeCfg.FallingSignLock=HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
	pDeadTimeCfg.FallingValue=10;                                            
	pDeadTimeCfg.Prescaler=HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;           
	pDeadTimeCfg.RisingLock=HRTIM_TIMDEADTIME_RISINGLOCK_READONLY;          
	pDeadTimeCfg.RisingSign=HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;         
	pDeadTimeCfg.RisingSignLock=HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;  
	pDeadTimeCfg.RisingValue=10;                                             
	HAL_HRTIM_DeadTimeConfig(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,&pDeadTimeCfg);
	
	pADCTriggerCfg.Trigger=HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;             
	pADCTriggerCfg.UpdateSource=HRTIM_ADCTRIGGERUPDATE_TIMER_A;             
	HAL_HRTIM_ADCTriggerConfig(&HRTIM1_structure,HRTIM_ADCTRIGGER_1,&pADCTriggerCfg);
	
	HAL_HRTIM_SimpleBaseStart(&HRTIM1_structure,HRTIM_TIMERINDEX_MASTER);   
	HAL_HRTIM_SimpleBaseStart(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A);  
	Delay_us(300);
	
	HAL_HRTIM_SimpleOCStart(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA1);
	HAL_HRTIM_SimpleOCStart(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA2);
	
	HAL_HRTIM_SimplePWMStart(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA1);
	HAL_HRTIM_SimplePWMStart(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA2);
	
	__HAL_HRTIM_TIMER_ENABLE_IT(&HRTIM1_structure,HRTIM_TIMERINDEX_TIMER_A,HRTIM_TIM_IT_REP);
	HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn,2,1); 
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);        
	
	__HAL_HRTIM_ENABLE(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A);       
	__HAL_HRTIM_ENABLE(&HRTIM1_structure, HRTIM_TIMERINDEX_MASTER);        
	
	HAL_HRTIM_WaveformSetOutputLevel(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_ACTIVE);
	HAL_HRTIM_WaveformSetOutputLevel(&HRTIM1_structure, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_ACTIVE);
	
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 500;
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 100; 
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 1000;
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP4xR = 100; 
}

