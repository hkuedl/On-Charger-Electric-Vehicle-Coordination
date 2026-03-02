

#include "state_machine.h"

CCMRAM __IO uint16_t Vin = 0;
CCMRAM __IO uint16_t Vout = 0;
CCMRAM __IO uint16_t Iout = 0;
CCMRAM __IO uint16_t Target_voltage = 5000;

CCMRAM __IO uint16_t Target_voltage_ov = 14300;
CCMRAM __IO uint16_t Target_current_oc = 5500;

CCMRAM __IO uint16_t V_temp = 0x00;
CCMRAM __IO uint16_t V_temp_1 = 0x00;
CCMRAM __IO uint16_t Vn = 0x00;
CCMRAM __IO uint16_t Vn_1 = 0x00;
CCMRAM __IO uint16_t Vn_2 = 0x00;
CCMRAM __IO uint16_t Vn_3 = 0x00;

__IO uint32_t Display_cnt = 0;  
extern uint8_t PC_command[50];

CCMRAM __IO float Vin_f = 0;    
CCMRAM __IO float Vout_f = 0;   
CCMRAM __IO float Vout1_f = 0;  
CCMRAM __IO float Iout_f = 0;   

CCMRAM DP_FlagStatus flag_Soft_start = START;
CCMRAM DP_FlagStatus flag_start_cnt = START;
CCMRAM DP_FlagStatus ERROR_flag = STOP;
CCMRAM DP_FlagStatus Data_update_flag = STOP;
CCMRAM __IO uint8_t i = 0;
CCMRAM __IO uint8_t j = 0;
CCMRAM __IO uint8_t k = 0;

extern __IO uint16_t ADC2_RESULT[3];

extern DMA_HandleTypeDef ADC2_DMA_Handler;

CCMRAM void Reset_VAR(void) {

    flag_Soft_start = START;
    flag_start_cnt = START;

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR =
        100;  
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 50;

    HRTIM1->sCommonRegs.ODISR |= HRTIM_OUTPUT_TA1;
    HRTIM1->sCommonRegs.ODISR |= HRTIM_OUTPUT_TA2;

    OLED_DISPLAY_INT();
    if (Vin_f > 24.0f) {
        OLED_ShowString(2, 6, "flag_Vin_ovp", 16);  
    } else {
        OLED_ShowString(2, 6, "Normal", 16);  
    }

    Vin = 0;
    Vout = 0;
    Iout = 0;

    Target_voltage = 5000;  

    Target_voltage_ov = 7500;
    
    Vin_f = 0;   
    Vout_f = 0;  
    Iout_f = 0;  

    V_temp = 0x00;
    V_temp_1 = 0x00;
    Vn = 0x00;
    Vn_1 = 0x00;
    Vn_2 = 0x00;
    Vn_3 = 0x00;

    flag_Soft_start = START;
    flag_start_cnt = START;
    ERROR_flag = STOP;
    i = 0;
    j = 0;
    k = 0;
    ADC2_RESULT[0] = 0x00;
    ADC2_RESULT[1] = 0x00;
    ADC2_RESULT[2] = 0x00;
    Display_cnt = 0;
}

typedef enum {
    Task_0_Initial_state,  
    Task_1_Get_ADC_VALUE,  
    Task_2_Vin_detc,       
    Task_3_Iout_detc,      
    Task_4_Soft_start      
} System_Task;

CCMRAM System_Task Current_State, Next_State;

void bcfsm(void)
	{
    
    type_3_tustin();

    type_3_int();

    flag_Soft_start = START;
    flag_start_cnt = STOP;  
    ERROR_flag = STOP;
    
    Target_voltage = 5000;  

    ADC2_RESULT[0] = 0x00;
    ADC2_RESULT[1] = 0x00;
    ADC2_RESULT[2] = 0x00;

    HRTIM1->sCommonRegs.OENR |= HRTIM_OUTPUT_TA1;
    HRTIM1->sCommonRegs.OENR |= HRTIM_OUTPUT_TA2;

    Green_ON();

	}

CCMRAM void HRTIM1_TIMA_IRQHandler(void) {
    if ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxDIER &
         HRTIM_TIM_IT_REP) == HRTIM_TIM_IT_REP) {
        
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxICR =
            HRTIM_TIM_IT_REP;

        if (__HAL_DMA_GET_FLAG(&ADC2_DMA_Handler, DMA_FLAG_TC3) != RESET)
            __HAL_DMA_CLEAR_FLAG(&ADC2_DMA_Handler, DMA_FLAG_TC3);

        Vn = (Vn_1 + Vn_2 + Vn_3 + ADC2_RESULT[0]) >> 2;
        Vn_1 = Vn_2;
        Vn_2 = Vn_3;
        Vn_3 = ADC2_RESULT[0];

        Vout_f = (float)((Vn * 3277) >> 12) * 0.0175f - 0.25f;
        Vout = Vout_f * 1000.0f;

        type_3_cal(Vout, V_temp_1);

        if (flag_Soft_start != STOP) {
            if (V_temp_1 > Target_voltage) {
                flag_Soft_start = STOP;  
            } else {
                V_temp_1 += 5;  
            }
        }
    }
}
