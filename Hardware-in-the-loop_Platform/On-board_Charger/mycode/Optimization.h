#ifndef __OPTIMIZATION_H__
#define __OPTIMIZATION_H__

#include "common.h"
#include "data.h"
#include "usart.h"

void optimization_init(void);
void time_step_update(uint16_t day, uint16_t step);
void load_data(void);
void state_update(void);
uint8_t run_optimization_cycle(void);

uint8_t run_optimization(float rho, float *dual_prices, float *upper_vars,
                        float energy_req, float max_power, 
                        uint16_t opt_duration, uint16_t coord_period,
                        float *power_result);

void update_power_energy(uint16_t hour, uint16_t period, float *price,
                        float *w_power_in, float *g_power_in,
                        float *y_energy_in, float *z_energy_in,
                        float energy_demand_in, float *upper_variable_in,
                        float rho_inloop_val, float rho_outloop_val);
void update_w_power(float *power_in, float *g_power_in, 
                   float power_max_val, uint16_t hour);
void update_z_energy(float *energy_in, float *y_energy_in, uint16_t hour);

float max_f(float a, float b);
float min_f(float a, float b);
float norm_vector(float *vec, uint16_t size);

#define MAX_ITER 1000
#define COMM_THRESHOLD 0.2f
#define STEPS_PER_DAY 96

extern __IO uint16_t current_day;
extern __IO uint16_t current_step;
extern __IO uint16_t duration;
extern __IO uint16_t period;
extern __IO float energy_demand;
extern __IO float power_max;
extern __IO uint8_t plug_status;
extern __IO uint8_t communicate_sign;
extern __IO uint8_t first_duration;

extern float upper_variable[MAX_HOUR];     
extern float dual[MAX_HOUR];               
extern float lower_variable[MAX_HOUR];
#endif
