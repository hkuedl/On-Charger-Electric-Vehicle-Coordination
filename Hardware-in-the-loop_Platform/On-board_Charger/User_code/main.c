

#include "main.h" 
#include "optimization.h" 
#include "data.h" 
#include "usart.h" 

extern void Initial_prepheral_(void); 

int main(void) 
{ 
    
    Initial_prepheral_(); 

    optimization_init(); 

    uint16_t simulation_day = 0;      
    uint16_t simulation_step = 0;     
    uint16_t max_days = 7;            

    HAL_Delay(1000); 

    while(simulation_day < max_days) 
    { 
        
        time_step_update(simulation_day, simulation_step); 

        uint8_t optimization_result = run_optimization_cycle(); 

        simulation_step++; 
        if(simulation_step >= STEPS_PER_DAY) { 
            simulation_step = 0; 
            simulation_day++; 
          
        } 

        HAL_Delay(1000); 
    }

}
