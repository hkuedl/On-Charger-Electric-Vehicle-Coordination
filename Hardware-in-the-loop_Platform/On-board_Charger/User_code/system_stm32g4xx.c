

#include "stm32g4xx.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE     8000000U 
#endif 

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U 
#endif 

#define VECT_TAB_OFFSET  0x00UL 

  uint32_t SystemCoreClock = HSI_VALUE;

  const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
  const uint8_t APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemInit(void)
{
  
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  
  #endif

#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; 
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; 
#endif
}

void SystemCoreClockUpdate(void)
{
  uint32_t tmp, pllvco, pllr, pllsource, pllm;

  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case 0x04:  
      SystemCoreClock = HSI_VALUE;
      break;

    case 0x08:  
      SystemCoreClock = HSE_VALUE;
      break;

    case 0x0C:  

      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1U ;
      if (pllsource == 0x02UL) 
      {
        pllvco = (HSI_VALUE / pllm);
      }
      else                   
      {
        pllvco = (HSE_VALUE / pllm);
      }
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1U) * 2U;
      SystemCoreClock = pllvco/pllr;
      break;

    default:
      break;
  }

  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  
  SystemCoreClock >>= tmp;
}

