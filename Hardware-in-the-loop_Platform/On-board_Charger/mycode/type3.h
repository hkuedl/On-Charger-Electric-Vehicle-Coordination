

#ifndef __type3__H__
#define __type3__H__

#include "common.h"

#define pF  (1e-12)
#define nF  (1e-9)
#define uF  (1e-6)

#define om   1.0
#define Kom  1000.0
#define Mom  1000000.0

#define nS   (1e-9)
#define uS   (1e-6)
#define ms   (1e-3)

void type_3_int(void);
void type_3_cal(uint16_t Vout,uint16_t Vref);
void type_3_tustin(void);

#endif
