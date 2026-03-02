#ifndef __DATA_H__
#define __DATA_H__

#include "stdint.h"

#define BASE_LOAD_SIZE 35037
#define LMP_PRICE_SIZE 34944
#define PV_GENERATION_SIZE 35037
#define EV_DATA_COUNT 401

typedef struct {
    uint16_t start_day;
    uint16_t start_step;
    uint16_t leave_day;
    uint16_t leave_step;
    float energy_demand;
    float power_limit;
} EV_Info;

extern const float base_load_data[BASE_LOAD_SIZE];
extern const float lmp_price_data[LMP_PRICE_SIZE];
extern const float pv_generation_data[PV_GENERATION_SIZE];
extern const EV_Info ev_data[EV_DATA_COUNT];

float get_base_load(uint16_t index);
float get_lmp_price(uint16_t index);
float get_pv_generation(uint16_t index);
EV_Info get_ev_info(uint16_t ev_id);
uint16_t get_ev_count(void);

#endif
