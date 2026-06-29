/**
 * @file    main.c
 * @brief   Main entry point for EV charging optimization simulation
 *
 * Runs a multi-day simulation where each EV is optimized for charging
 * based on LMP prices and charging constraints. The optimizer runs
 * at every 15-minute time step (96 steps per day).
 */

#include "main.h" 
#include "optimization.h" 
#include "data.h" 
#include "usart.h" 

extern void Initial_prepheral_(void); 

int main(void) 
{ 
    /* Initialize hardware peripherals (UART, timers, ADC, etc.) */
    Initial_prepheral_(); 

    /* Initialize optimization module */
    optimization_init(); 

    /* Simulation parameters */
    uint16_t simulation_day = 0;       ///< Current simulation day (0-based)
    uint16_t simulation_step = 0;      ///< Current step within the day (0-95)
    uint16_t max_days = 7;             ///< Total simulation duration in days

    /* Allow hardware to settle */
    HAL_Delay(1000); 

    /* Main simulation loop: iterate over days and time steps */
    while(simulation_day < max_days) 
    { 
        /* Update global time state */
        time_step_update(simulation_day, simulation_step); 

        /* Run one optimization cycle for current time step */
        uint8_t optimization_result = run_optimization_cycle(); 

        /* Advance to next time step */
        simulation_step++; 
        if(simulation_step >= STEPS_PER_DAY) { 
            simulation_step = 0; 
            simulation_day++;          
        } 

        /* Delay to simulate real-time progression (1 second per step) */
        HAL_Delay(1000); 
    }

    /* Simulation complete; system enters idle state */
}