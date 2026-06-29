/**
 * @file    optimization.c
 * @brief   EV charging optimizer using ADMM (Alternating Direction Method of Multipliers)
 *
 * This module solves a convex quadratic optimization problem for EV charging:
 *   minimize: sum_t [ alpha * power[t]^2 + beta * energy[t] + price[t] * power[t] ]
 *   subject to:
 *     energy[t+1] = A * energy[t] + B * power[t]    (energy dynamics)
 *     0 <= power[t] <= P_max                          (power limits)
 *     energy[0] = E_demand                            (initial energy)
 *     energy[horizon] = 0                             (final energy)
 *
 * The problem is decomposed using ADMM into:
 *   - An inner DP subproblem (convex QP solvable via Riccati recursion)
 *   - Projection onto constraints (box constraints on power)
 *   - Dual variable updates (Lagrange multiplier updates)
 *
 * rho adaptation follows the standard ADMM heuristic:
 *   - Increase rho if primal residual is large (slow primal convergence)
 *   - Decrease rho if dual residual is large (slow dual convergence)
 */

#include "optimization.h" 
#include "math.h" 
#include "string.h" 
#include "usart.h"

/* ============================================================================
 * Global State Variables (I/O volatile for ISR access)
 * ============================================================================ */
__IO uint16_t current_day = 0;           ///< Current simulation day index
__IO uint16_t current_step = 0;          ///< Current step within the day
__IO uint16_t duration = 0;              ///< Charging duration in hours
__IO uint16_t period = 12;               ///< Coordination period (hours)
__IO float energy_demand = 0.0f;         ///< Total energy required by EV (kWh)
__IO float power_max = 0.0f;             ///< Maximum charging power (kW)
__IO uint8_t plug_status = 0;            ///< 1 = EV plugged in, 0 = disconnected
__IO uint8_t communicate_sign = 1;       ///< Communication status flag
__IO uint8_t first_duration = 1;         ///< Flag indicating first optimization run

/* ============================================================================
 * CCMRAM Buffer Arrays (fast RAM for real-time computation)
 * ============================================================================ */
CCMRAM float power[MAX_HOUR];              ///< Optimized power schedule
CCMRAM float energy[MAX_HOUR+1];           ///< Energy state trajectory
CCMRAM float upper_variable[MAX_HOUR];     ///< Upper bound variable for pricing
CCMRAM float dual[MAX_HOUR];               ///< Dual variable (Lagrange multipliers)
CCMRAM float lower_variable[MAX_HOUR];     ///< Lower bound variable
CCMRAM float power_old[MAX_HOUR];          ///< Previous iteration power

/* ============================================================================
 * ADMM Algorithm Parameters
 * ============================================================================ */
CCMRAM float rho_outloop = 0.1f;     ///< Outer-loop penalty parameter
CCMRAM float rho_inloop = 0.05f;     ///< Inner-loop penalty parameter
CCMRAM float mu = 10.0f;             ///< Primal/dual residual ratio threshold
CCMRAM float tau = 2.0f;             ///< rho scaling factor for adaptation
CCMRAM float rho_min = 1e-3f;        ///< Minimum allowed rho
CCMRAM float rho_max = 1e3f;         ///< Maximum allowed rho
CCMRAM float primal_tol = 1e-3f;     ///< Primal residual convergence tolerance
CCMRAM float dual_tol = 1e-3f;       ///< Dual residual convergence tolerance

/* ============================================================================
 * DP Solution Vectors (Riccati recursion)
 * ============================================================================ */
CCMRAM float P[MAX_HOUR+1];          ///< Cost-to-go quadratic coefficient
CCMRAM float p[MAX_HOUR+1];          ///< Cost-to-go affine term
CCMRAM float K[MAX_HOUR];            ///< Feedback gain matrix
CCMRAM float d[MAX_HOUR];            ///< Feedforward offset
CCMRAM float Q[MAX_HOUR];            ///< Running cost quadratic coefficient
CCMRAM float R[MAX_HOUR];            ///< Running cost cross term
CCMRAM float q_k[MAX_HOUR];          ///< Running cost affine term
CCMRAM float r_k[MAX_HOUR];          ///< Running cost linear term
CCMRAM float w_power[MAX_HOUR];      ///< Auxiliary variable for power (split)
CCMRAM float z_energy[MAX_HOUR+1];   ///< Auxiliary variable for energy (split)
CCMRAM float y_energy[MAX_HOUR+1];   ///< Dual variable for energy balance
CCMRAM float g_power[MAX_HOUR];      ///< Dual variable for power (Lagrange mult.)

/* ============================================================================
 * System Dynamics Parameters
 * ============================================================================ */
CCMRAM float A = 1.0f;               ///< Energy dynamics coefficient (discharge rate)
CCMRAM float B = -0.95f;             ///< Power dynamics coefficient (charging efficiency)
CCMRAM float alpha = 0.1f;           ///< Quadratic cost coefficient for power
CCMRAM float betaa = 0.5f;           ///< Linear cost coefficient for energy
CCMRAM float delta = 1.0f;           ///< Scaling factor for power output

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * Return the maximum of two floating-point values.
 */
CCMRAM float max_f(float a, float b) { 
    return (a > b) ? a : b; 
} 

/**
 * Return the minimum of two floating-point values.
 */
CCMRAM float min_f(float a, float b) { 
    return (a < b) ? a : b; 
} 

/**
 * Compute the L2 Euclidean norm of a vector.
 *
 * @param vec   Pointer to vector data
 * @param size  Number of elements in the vector
 * @return      sqrt(sum_i(vec[i]^2))
 */
CCMRAM float norm_vector(float *vec, uint16_t size) { 
    float sum = 0.0f; 
    for(uint16_t i = 0; i < size; i++) { 
        sum += vec[i] * vec[i]; 
    } 
    return sqrtf(sum);
}

/* ============================================================================
 * ADMM Subproblem Solver: Dynamic Programming with Riccati Recursion
 * ============================================================================
 * 
 * Solves the inner QP subproblem at each ADMM iteration:
 *   min_{power[0..T-1]} sum_t [ alpha*p[t]^2 + beta*e[t] + price[t]*p[t]
 *                              + (rho/2)*||p[t]-w_p[t]+g_p[t]/rho||^2
 *                              + (rho/2)*(e[t]-z_e[t]+y_e[t]/rho)^2 ]
 *   s.t.   e[t+1] = A*e[t] + B*p[t]
 *          e[0] = E_demand
 *
 * Uses backward Riccati recursion to compute gain K[t] and offset d[t],
 * then forward sweep to compute optimal power.
 */
CCMRAM void update_power_energy(uint16_t hour, uint16_t period, float *price,
                               float *w_power_in, float *g_power_in,
                               float *y_energy_in, float *z_energy_in,
                               float energy_demand_in, float *upper_variable_in,
                               float rho_inloop_val, float rho_outloop_val) { 
    
    uint16_t t; 
    float Q_f, q_f; 
    float P_next, p_next, denom; 

    /* Set terminal cost coefficients */
    Q_f = rho_inloop_val; 
    q_f = betaa + rho_inloop_val * (y_energy_in[hour] - z_energy_in[hour]); 

    /* Backward pass: set stage cost coefficients for each time step */
    for(t = 0; t < hour; t++) { 
        Q[t] = rho_inloop_val; 
        R[t] = alpha * 2.0f + rho_inloop_val; 
        q_k[t] = rho_inloop_val * (y_energy_in[t] - z_energy_in[t]); 
        r_k[t] = price[t] + rho_inloop_val * (g_power_in[t] - w_power_in[t]); 

        /* Adjust for pricing constraint in coordination period */
        if(t < period) { 
            r_k[t] = r_k[t] - rho_outloop_val * upper_variable_in[t] * delta; 
            R[t] = R[t] + rho_outloop_val; 
        } 
    } 

    /* Set terminal cost coefficients for Riccati recursion */
    P[hour] = Q_f; 
    p[hour] = q_f; 

    /* Backward Riccati recursion: compute P[t] and p[t] from t=hour-1 down to 0 */
    for(t = hour; t > 0; t--) { 
        uint16_t idx = t - 1; 
        P_next = P[t]; 
        p_next = p[t]; 

        /* Compute denominator for Riccati update (avoid division by zero) */
        denom = R[idx] + B * B * P_next; 
        if(fabsf(denom) < 1e-12f) { 
            denom = (denom >= 0) ? 1e-12f : -1e-12f; 
        } 

        /* Compute feedback gain K[idx] and feedforward offset d[idx] */
        K[idx] = (A * B * P_next) / denom; 
        d[idx] = (B * p_next + r_k[idx]) / denom; 

        /* Update Riccati matrices for previous time step */
        P[idx] = Q[idx] + A * A * P_next - 
                 (A * B * P_next) * (A * B * P_next) / denom; 
        p[idx] = q_k[idx] + A * (p_next - B * P_next * d[idx]); 
    } 

    /* Set initial energy state */
    energy[0] = energy_demand_in; 
    
    /* Forward sweep: compute optimal power and energy trajectories */
    for(t = 0; t < hour; t++) { 
        power[t] = -K[t] * energy[t] - d[t];     /* Optimal power at time t */
        energy[t+1] = A * energy[t] + B * power[t];  /* Propagate energy state */
    } 
} 

/* ============================================================================
 * ADMM Projection: Box Constraints on Power
 * ============================================================================ */
CCMRAM void update_w_power(float *power_in, float *g_power_in, 
                          float power_max_val, uint16_t hour) { 
    uint16_t t; 
    for(t = 0; t < hour; t++) { 
        /* Compute augmented variable w_power = power + g_power/rho */
        w_power[t] = power_in[t] + g_power_in[t]; 
        /* Apply upper bound constraint */
        w_power[t] = min_f(w_power[t], power_max_val * delta);  
        /* Apply lower bound constraint */
        w_power[t] = max_f(w_power[t], 0.0f);                   
    } 
} 

/* ============================================================================
 * ADMM Projection: Non-negativity Constraint on Energy
 * ============================================================================ */
CCMRAM void update_z_energy(float *energy_in, float *y_energy_in, 
                           uint16_t hour) { 
    uint16_t t; 
    for(t = 0; t <= hour; t++) { 
        /* Compute augmented variable z_energy = energy + y_energy/rho */
        z_energy[t] = energy_in[t] + y_energy_in[t]; 
        /* Clamp to non-negative values */
        z_energy[t] = max_f(z_energy[t], 0.0f);  
    } 
}

/* ============================================================================
 * Main ADMM Solver Loop
 * ============================================================================
 * 
 * Iteratively solves the optimization problem using ADMM:
 *   1. Solve inner DP subproblem (update_power_energy)
 *   2. Project onto constraints (update_w_power, update_z_energy)
 *   3. Update dual variables (g_power, y_energy)
 *   4. Check primal and dual residuals for convergence
 *   5. Adapt rho based on residual ratio
 */
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

    /* Initialize ADMM parameters */
    rho_outloop = rho; 
    rho_inloop = 0.05f; 

    /* Reset all buffers to zero */
    memset(power, 0, sizeof(power)); 
    memset(energy, 0, sizeof(energy)); 
    memset(w_power, 0, sizeof(w_power)); 
    memset(z_energy, 0, sizeof(z_energy)); 
    memset(y_energy, 0, sizeof(y_energy)); 
    memset(g_power, 0, sizeof(g_power)); 

    /* Main ADMM iteration loop */
    for(iteration = 0; iteration < MAX_ITER; iteration++) { 
        
        /* Save previous iterates for dual residual computation */
        memcpy(w_power_old, w_power, sizeof(w_power)); 
        memcpy(z_energy_old, z_energy, sizeof(z_energy)); 

        /* Step 1: Solve inner QP subproblem via Riccati DP */
        update_power_energy(opt_duration, coord_period, dual_prices, w_power, g_power, 
                           y_energy, z_energy, energy_req, upper_vars, 
                           rho_inloop, rho_outloop); 
        
        /* Step 2: Project power onto box constraints */
        update_w_power(power, g_power, max_power, opt_duration); 
        
        /* Step 3: Project energy onto non-negativity constraints */
        update_z_energy(energy, y_energy, opt_duration); 

        /* Step 4: Update dual variables (Lagrange multipliers) */
        for(t = 0; t < opt_duration; t++) { 
            g_power[t] += power[t] - w_power[t];  /* Power dual update */
        } 
        for(t = 0; t <= opt_duration; t++) { 
            y_energy[t] += energy[t] - z_energy[t];  /* Energy dual update */
        } 

        /* Step 5: Compute primal residual ||power - w_power|| + ||energy - z_energy|| */
        for(t = 0; t < opt_duration; t++) { 
            primal_res_vec[t] = power[t] - w_power[t]; 
        } 
        for(t = 0; t <= opt_duration; t++) { 
            primal_res_vec[opt_duration + t] = energy[t] - z_energy[t]; 
        } 
        primal_residual = norm_vector(primal_res_vec, opt_duration + opt_duration + 1); 

        /* Step 6: Compute dual residual ||w_power_new - w_power_old|| + ||z_energy_new - z_energy_old|| */
        for(t = 0; t < opt_duration; t++) { 
            dual_res_vec[t] = power[t] - w_power_old[t]; 
        } 
        for(t = 0; t <= opt_duration; t++) { 
            dual_res_vec[opt_duration + t] = energy[t] - z_energy_old[t]; 
        } 
        dual_residual = rho_inloop * norm_vector(dual_res_vec, opt_duration + opt_duration + 1); 

        /* Step 7: Adapt rho based on primal/dual residual ratio */
        if(iteration > 0 && (iteration % 20 == 0)) { 
            if(primal_residual > mu * dual_residual) { 
                /* Primal convergence slow -> increase rho */
                rho_inloop = min_f(tau * rho_inloop, rho_max); 
            } else if(dual_residual > mu * primal_residual) { 
                /* Dual convergence slow -> decrease rho */
                rho_inloop = max_f(rho_inloop / tau, rho_min); 
            } 
        } 

        /* Step 8: Check convergence */
        if(primal_residual < primal_tol && dual_residual < dual_tol) { 
            
            /* Return optimal power schedule (scaled by delta) */
            for(t = 0; t < opt_duration; t++) { 
                power_result[t] = max_f(0.0f, power[t] / delta); 
            } 
            return 1;  /* Converged */
        } 
    } 

    /* Return best available solution if not converged */
    for(t = 0; t < opt_duration; t++) { 
        power_result[t] = max_f(0.0f, power[t] / delta); 
    } 

    return 0;  /* Did not converge within MAX_ITER */
}

/* ============================================================================
 * Time Step Management
 * ============================================================================ */
void time_step_update(uint16_t day, uint16_t step) { 
    current_day = day; 
    current_step = step; 
} 

/* ============================================================================
 * Data Loading: Find EV Session for Current Time
 * ============================================================================ */
void load_data(void) { 
    
    energy_demand = 0.0f; 
    power_max = 0.0f; 

    /* Get total number of connected EVs */
    uint16_t ev_count = get_ev_count(); 
    
    /* Find the EV that starts charging at current day/step */
    for(uint16_t i = 0; i < ev_count; i++) { 
        EV_Info ev_info = get_ev_info(i); 
        
        if(current_day == ev_info.start_day && current_step == ev_info.start_step) { 
            /* Calculate remaining charging duration in hours */
            duration = (ev_info.leave_day - ev_info.start_day) * STEPS_PER_DAY + 
                      (ev_info.leave_step - ev_info.start_step); 
            
            /* Load EV charging requirements */
            energy_demand = ev_info.energy_demand; 
            power_max = ev_info.power_limit; 
            
            break; 
        } 
    } 
} 

/* ============================================================================
 * State Update: Detect Plug-in, Fetch Prices, Initialize Buffers
 * ============================================================================ */
void state_update(void) { 
    uint16_t record = current_day * STEPS_PER_DAY + current_step; 
    
    /* If no active session, try to load data for a new EV */
    if(duration == 0) { 
        load_data(); 
        first_duration = 1; 
    } 
    
    /* Process charging session if EV is plugged in */
    if(duration > 0) { 
        plug_status = 1; 

        /* Zero out optimization buffers */
        memset(power, 0, sizeof(float) * duration); 
        memset(upper_variable, 0, sizeof(float) * duration); 

        /* Fetch LMP (Locational Marginal Price) for the optimization horizon */
        for(uint16_t i = 0; i < duration && (record + i) < LMP_PRICE_SIZE; i++) { 
            dual[i] = get_lmp_price(record + i); 
        } 

        /* Copy previous power schedule for warm-start */
        memcpy(power_old, power, sizeof(float) * duration); 
        
        /* Shift lower variable for warm-start initialization */
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
        /* No EV plugged in */
        plug_status = 0; 
    } 
    
    communicate_sign = 1; 
} 

/* ============================================================================
 * Main Optimization Cycle: Orchestrates state update and ADMM solving
 * ============================================================================ */
uint8_t run_optimization_cycle(void) { 

    /* Update state and fetch prices */
    state_update(); 

    /* Process UART data for inter-node communication */
    process_uart_data(); 
    
    /* Skip optimization if no EV is plugged in */
    if(plug_status == 0 || duration == 0) { 
        return 0; 
    } 

    float rho = 1.0f; 
    float power_result[MAX_HOUR]; 

    /* Run the ADMM solver */
    uint8_t success = run_optimization( 
        rho,                     /* Penalty parameter */
        dual,                    /* Price signal (dual variable) */
        upper_variable,          /* Upper bound variable */
        energy_demand,           /* Total energy required */
        power_max,               /* Maximum charging power */
        duration,                /* Optimization horizon */
        period,                  /* Coordination period */
        power_result             /* Output: optimal power schedule */
    ); 
    
    /* If converged, apply the solution and trigger communication */
    if(success) { 
        for(uint16_t i = 0; i < duration; i++) { 
            power[i] = power_result[i]; 
        } 

        event_triggered_communicate(power, duration); 

        if(duration > 0) { 
            duration--;  /* Advance charging duration */
        } 

        return 1; 
    } else { 
        return 0; 
    } 
} 

/* ============================================================================
 * Initialization: Reset All State and Buffers
 * ============================================================================ */
CCMRAM void optimization_init(void) { 

    /* Zero all optimization buffers */
    memset(power, 0, sizeof(power)); 
    memset(energy, 0, sizeof(energy)); 
    memset(upper_variable, 0, sizeof(upper_variable)); 
    memset(dual, 0, sizeof(dual)); 
    memset(lower_variable, 0, sizeof(lower_variable)); 
    memset(power_old, 0, sizeof(power_old)); 

    /* Reset global state variables */
    current_day = 0; 
    current_step = 0; 
    duration = 0; 
    period = 12; 
    energy_demand = 0.0f; 
    power_max = 0.0f; 
    plug_status = 0; 
    communicate_sign = 1; 
    first_duration = 1; 

    /* Initialize ADMM parameters */
    rho_outloop = 0.1f; 
    rho_inloop = 0.05f; 
    mu = 10.0f; 
    tau = 2.0f; 
    rho_min = 1e-3f; 
    rho_max = 1e3f; 
    primal_tol = 1e-3f; 
    dual_tol = 1e-3f; 

    /* Initialize system dynamics parameters */
    A = 1.0f; 
    B = -0.95f; 
    alpha = 0.1f; 
    betaa = 0.5f; 
    delta = 1.0f; 

}