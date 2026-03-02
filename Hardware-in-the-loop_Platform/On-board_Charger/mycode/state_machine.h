

#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include "common.h"

void bcfsm(void);
void Set_duty(uint16_t Time_width);         
void Voltage_Loop(void);
void MODS_01H(void);
void MODS_06H(void);
#define PWM_PERIOD  27200

typedef enum {STOP = 0, START = !STOP} DP_FlagStatus;

#endif
