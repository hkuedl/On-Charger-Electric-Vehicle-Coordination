

#include "optimization.h" 
#include "math.h" 
#include "string.h" 
#include "usart.h"

__IO uint16_t current_day = 0;           
__IO uint16_t current_step = 0;          
__IO uint16_t duration = 0;              
__IO uint16_t period = 12;               
__IO float energy_demand = 0.0f;         
__IO float power_max = 0.0f;             
__IO uint8_t plug_status = 0;            
__IO uint8_t communicate_sign = 1;       
__IO uint8_t first_duration = 1;         

CCMRAM float power[MAX_HOUR];              
CCMRAM float energy[MAX_HOUR+1];           
CCMRAM float upper_variable[MAX_HOUR];     
CCMRAM float dual[MAX_HOUR];               
CCMRAM float lower_variable[MAX_HOUR];     
CCMRAM float power_old[MAX_HOUR];          

CCMRAM float rho_outloop = 0.1f;     
CCMRAM float rho_inloop = 0.05f;     
CCMRAM float mu = 10.0f;             
CCMRAM float tau = 2.0f;             
CCMRAM float rho_min = 1e-3f;        
CCMRAM float rho_max = 1e3f;         
CCMRAM float primal_tol = 1e-3f;     
CCMRAM float dual_tol = 1e-3f;       

CCMRAM float P[MAX_HOUR+1];          
CCMRAM float p[MAX_HOUR+1];          
CCMRAM float K[MAX_HOUR];            
CCMRAM float d[MAX_HOUR];            
CCMRAM float Q[MAX_HOUR];            
CCMRAM float R[MAX_HOUR];            
CCMRAM float q_k[MAX_HOUR];          
CCMRAM float r_k[MAX_HOUR];          
CCMRAM float w_power[MAX_HOUR];      
CCMRAM float z_energy[MAX_HOUR+1];   
CCMRAM float y_energy[MAX_HOUR+1];   
CCMRAM float g_power[MAX_HOUR];      

CCMRAM float A = 1.0f;               
CCMRAM float B = -0.95f;             
CCMRAM float alpha = 0.1f;           
CCMRAM float betaa = 0.5f;           
CCMRAM float delta = 1.0f;           

CCMRAM float max_f(float a, float b) { 
    return (a > b) ? a : b; 
} 

CCMRAM float min_f(float a, float b) { 
    return (a < b) ? a : b; 
} 

CCMRAM float norm_vector(float *vec, uint16_t size) { 
    float sum = 0.0f; 
    for(uint16_t i = 0; i < size; i++) { 
        sum += vec[i] * vec[i]; 
    } 
    return sqrtf(sum); 
}

CCMRAM void update_power_energy(uint16_t hour, uint16_t period, float *price,
                               float *w_power_in, float *g_power_in,
                               float *y_energy_in, float *z_energy_in,
                               float energy_demand_in, float *upper_variable_in,
                               float rho_inloop_val, float rho_outloop_val) { 
    
    uint16_t t; 
    float Q_f, q_f; 
    float P_next, p_next, denom; 

    Q_f = rho_inloop_val; 
    q_f = betaa + rho_inloop_val * (y_energy_in[hour] - z_energy_in[hour]); 

    for(t = 0; t < hour; t++) { 
        Q[t] = rho_inloop_val; 
        R[t] = alpha * 2.0f + rho_inloop_val; 
        q_k[t] = rho_inloop_val * (y_energy_in[t] - z_energy_in[t]); 
        r_k[t] = price[t] + rho_inloop_val * (g_power_in[t] - w_power_in[t]); 

        if(t < period) { 
            r_k[t] = r_k[t] - rho_outloop_val * upper_variable_in[t] * delta; 
            R[t] = R[t] + rho_outloop_val; 
        } 
    } 

    P[hour] = Q_f; 
    p[hour] = q_f; 

    for(t = hour; t > 0; t--) { 
        uint16_t idx = t - 1; 
        P_next = P[t]; 
        p_next = p[t]; 

        denom = R[idx] + B * B * P_next; 
        if(fabsf(denom) < 1e-12f) { 
            denom = (denom >= 0) ? 1e-12f : -1e-12f; 
        } 

        K[idx] = (A * B * P_next) / denom; 
        d[idx] = (B * p_next + r_k[idx]) / denom; 

        P[idx] = Q[idx] + A * A * P_next - 
                 (A * B * P_next) * (A * B * P_next) / denom; 
        p[idx] = q_k[idx] + A * (p_next - B * P_next * d[idx]); 
    } 

    energy[0] = energy_demand_in; 
    
    for(t = 0; t < hour; t++) { 
        power[t] = -K[t] * energy[t] - d[t];     
        energy[t+1] = A * energy[t] + B * power[t]; 
    } 
} 

CCMRAM void update_w_power(float *power_in, float *g_power_in, 
                          float power_max_val, uint16_t hour) { 
    uint16_t t; 
    for(t = 0; t < hour; t++) { 
        w_power[t] = power_in[t] + g_power_in[t]; 
        w_power[t] = min_f(w_power[t], power_max_val * delta);  
        w_power[t] = max_f(w_power[t], 0.0f);                   
    } 
} 

CCMRAM void update_z_energy(float *energy_in, float *y_energy_in, 
                           uint16_t hour) { 
    uint16_t t; 
    for(t = 0; t <= hour; t++) { 
        z_energy[t] = energy_in[t] + y_energy_in[t]; 
        z_energy[t] = max_f(z_energy[t], 0.0f);  
    } 
}

uint8_t run_optimization(float rho, float *dual_prices, float *upper_vars,
                        float energy_req, float max_power, 
                        uint16_t opt_duration, uint16_t coord_period,
                        float *power_result) { 
    
    uint16_t iteration, t; 
    float primal_residual, dual_residual; 
    float w_power_old[MAX_HOUR]; 
    float z_energy_old[MAX_HOUR+1]; 
    float primal_res_vec[MAX_HOUR + MAX_HOUR + 1]; 
    float dual_res_vec[MAX_HOUR + MAX_HOUR + 1]; 

    rho_outloop = rho; 
    rho_inloop = 0.05f; 

    memset(power, 0, sizeof(power)); 
    memset(energy, 0, sizeof(energy)); 
    memset(w_power, 0, sizeof(w_power)); 
    memset(z_energy, 0, sizeof(z_energy)); 
    memset(y_energy, 0, sizeof(y_energy)); 
    memset(g_power, 0, sizeof(g_power)); 

    for(iteration = 0; iteration < MAX_ITER; iteration++) { 
        
        memcpy(w_power_old, w_power, sizeof(w_power)); 
        memcpy(z_energy_old, z_energy, sizeof(z_energy)); 

        update_power_energy(opt_duration, coord_period, dual_prices, w_power, g_power, 
                           y_energy, z_energy, energy_req, upper_vars, 
                           rho_inloop, rho_outloop); 
        
        update_w_power(power, g_power, max_power, opt_duration); 
        update_z_energy(energy, y_energy, opt_duration); 

        for(t = 0; t < opt_duration; t++) { 
            g_power[t] += power[t] - w_power[t]; 
        } 
        for(t = 0; t <= opt_duration; t++) { 
            y_energy[t] += energy[t] - z_energy[t]; 
        } 

        for(t = 0; t < opt_duration; t++) { 
            primal_res_vec[t] = power[t] - w_power[t]; 
        } 
        for(t = 0; t <= opt_duration; t++) { 
            primal_res_vec[opt_duration + t] = energy[t] - z_energy[t]; 
        } 
        primal_residual = norm_vector(primal_res_vec, opt_duration + opt_duration + 1); 

        for(t = 0; t < opt_duration; t++) { 
            dual_res_vec[t] = power[t] - w_power_old[t]; 
        } 
        for(t = 0; t <= opt_duration; t++) { 
            dual_res_vec[opt_duration + t] = energy[t] - z_energy_old[t]; 
        } 
        dual_residual = rho_inloop * norm_vector(dual_res_vec, opt_duration + opt_duration + 1); 

        if(iteration > 0 && (iteration % 20 == 0)) { 
            if(primal_residual > mu * dual_residual) { 
                rho_inloop = min_f(tau * rho_inloop, rho_max); 
            } else if(dual_residual > mu * primal_residual) { 
                rho_inloop = max_f(rho_inloop / tau, rho_min); 
            } 
        } 

        if(primal_residual < primal_tol && dual_residual < dual_tol) { 
            
            for(t = 0; t < opt_duration; t++) { 
                power_result[t] = max_f(0.0f, power[t] / delta); 
            } 
            
            return 1;  
        } 
    } 

    for(t = 0; t < opt_duration; t++) { 
        power_result[t] = max_f(0.0f, power[t] / delta); 
    } 

    return 0;  
}

void time_step_update(uint16_t day, uint16_t step) { 
    current_day = day; 
    current_step = step; 
} 

void load_data(void) { 
    
    energy_demand = 0.0f; 
    power_max = 0.0f; 

    uint16_t ev_count = get_ev_count(); 
    
    for(uint16_t i = 0; i < ev_count; i++) { 
        EV_Info ev_info = get_ev_info(i); 
        
        if(current_day == ev_info.start_day && current_step == ev_info.start_step) { 
            
            duration = (ev_info.leave_day - ev_info.start_day) * STEPS_PER_DAY + 
                      (ev_info.leave_step - ev_info.start_step); 
            
            energy_demand = ev_info.energy_demand; 
            power_max = ev_info.power_limit; 
            
            break; 
        } 
    } 
} 

void state_update(void) { 
    uint16_t record = current_day * STEPS_PER_DAY + current_step; 
    
    if(duration == 0) { 
        load_data(); 
        first_duration = 1; 
    } 
    
    if(duration > 0) { 
        
        plug_status = 1; 

        memset(power, 0, sizeof(float) * duration); 

        memset(upper_variable, 0, sizeof(float) * duration); 

        for(uint16_t i = 0; i < duration && (record + i) < LMP_PRICE_SIZE; i++) { 
            dual[i] = get_lmp_price(record + i); 
        } 

        memcpy(power_old, power, sizeof(float) * duration); 
        
        if(first_duration) { 
            
            memset(lower_variable, 0, sizeof(float) * duration); 
            first_duration = 0; 
        } else { 
            
            for(uint16_t i = 0; i < duration - 1; i++) { 
                lower_variable[i] = lower_variable[i + 1]; 
            } 
            if(duration > 0) { 
                lower_variable[duration - 1] = 0.0f; 
            } 
        } 
    } else { 
        plug_status = 0; 
    } 
    
    communicate_sign = 1; 
} 

uint8_t run_optimization_cycle(void) { 

    state_update(); 

    process_uart_data(); 
    
    if(plug_status == 0 || duration == 0) { 
        return 0; 
    } 

    float rho = 1.0f; 
    float power_result[MAX_HOUR]; 

    uint8_t success = run_optimization( 
        rho,                    
        dual,                   
        upper_variable,         
        energy_demand,          
        power_max,              
        duration,               
        period,                 
        power_result            
    ); 
    
    if(success) { 
        
        for(uint16_t i = 0; i < duration; i++) { 
            power[i] = power_result[i]; 
        } 

        event_triggered_communicate(power, duration); 

        if(duration > 0) { 
            duration--; 
        } 

        return 1; 
    } else { 
        return 0; 
    } 
} 

CCMRAM void optimization_init(void) { 

    memset(power, 0, sizeof(power)); 
    memset(energy, 0, sizeof(energy)); 
    memset(upper_variable, 0, sizeof(upper_variable)); 
    memset(dual, 0, sizeof(dual)); 
    memset(lower_variable, 0, sizeof(lower_variable)); 
    memset(power_old, 0, sizeof(power_old)); 

    current_day = 0; 
    current_step = 0; 
    duration = 0; 
    period = 12; 
    energy_demand = 0.0f; 
    power_max = 0.0f; 
    plug_status = 0; 
    communicate_sign = 1; 
    first_duration = 1; 

    rho_outloop = 0.1f; 
    rho_inloop = 0.05f; 
    mu = 10.0f; 
    tau = 2.0f; 
    rho_min = 1e-3f; 
    rho_max = 1e3f; 
    primal_tol = 1e-3f; 
    dual_tol = 1e-3f; 

    A = 1.0f; 
    B = -0.95f; 
    alpha = 0.1f; 
    betaa = 0.5f; 
    delta = 1.0f; 

}
