/**
 * @file    optimization.h
 * @brief   Header for EV charging optimization module
 *          Implements ADMM-based distributed optimization for charging scheduling
 */
#ifndef __OPTIMIZATION_H__
#define __OPTIMIZATION_H__

#include "common.h"
#include "data.h"
#include "usart.h"

/**
 * Initialize optimization module and reset all internal state variables.
 * Called once at startup before any optimization cycles.
 */
void optimization_init(void);

/**
 * Update the current time step (day and step index).
 * @param day   Current simulation day index
 * @param step  Current time step index within the day
 */
void time_step_update(uint16_t day, uint16_t step);

/**
 * Load data for the current EV session (energy demand, power limit, duration).
 * Scans all connected EVs to find the one that starts charging at current time.
 */
void load_data(void);

/**
 * Update the charging state: detect plug-in, fetch LMP prices, initialize
 * optimization buffers. Resets state when no EV is plugged in.
 */
void state_update(void);

/**
 * Main entry point for one optimization cycle.
 * Calls state_update(), then invokes the ADMM solver, and returns the
 * computed optimal power schedule.
 * @return  1 if optimization converges, 0 otherwise
 */
uint8_t run_optimization_cycle(void);

/**
 * Core ADMM solver for the charging optimization problem.
 * Solves a convex quadratic program over the horizon [0, duration) using
 * Alternating Direction Method of Multipliers.
 *
 * @param rho           Primal penalty parameter (regularization weight)
 * @param dual_prices   Price signal (dual variable) for each hour
 * @param upper_vars    Upper bound variable for power constraint
 * @param energy_req    Total energy required by the EV (kWh)
 * @param max_power     Maximum charging power per hour (kW)
 * @param opt_duration  Number of hours in the optimization window
 * @param coord_period  Coordination period for pricing (hours)
 * @param power_result  Output buffer for optimal power schedule
 * @return              1 if converged within tolerance, 0 if exceeded iterations
 */
uint8_t run_optimization(float rho, float *dual_prices, float *upper_vars,
                        float energy_req, float max_power, 
                        uint16_t opt_duration, uint16_t coord_period,
                        float *power_result);

/**
 * Compute the optimal power schedule using dynamic programming with
 * ADMM decomposition. Solves the DP backward to get Riccati gain K[t]
 * and affine offset d[t], then forward to compute power[t].
 *
 * @param hour            Current horizon length being solved
 * @param period          Coordination period for pricing
 * @param price           Hourly electricity price vector
 * @param w_power_in      Auxiliary variable for power (primal split)
 * @param g_power_in      Dual variable (Lagrange multiplier) for power
 * @param y_energy_in     Dual variable for energy balance
 * @param z_energy_in     Auxiliary variable for energy (primal split)
 * @param energy_demand_in Total energy required
 * @param upper_variable_in Upper bound variable for pricing constraint
 * @param rho_inloop_val  Inner-loop rho parameter
 * @param rho_outloop_val Outer-loop rho parameter
 */
void update_power_energy(uint16_t hour, uint16_t period, float *price,
                        float *w_power_in, float *g_power_in,
                        float *y_energy_in, float *z_energy_in,
                        float energy_demand_in, *upper_variable_in,
                        float rho_inloop_val, float rho_outloop_val);

/**
 * Clamp the auxiliary power variable w_power within feasible bounds:
 *   [0, power_max * delta]
 * @param power_in        Raw power values
 * @param g_power_in      Dual variable for power
 * @param power_max_val   Maximum allowable power
 * @param hour            Number of hours to process
 */
void update_w_power(float *power_in, float *g_power_in, 
                   float power_max_val, uint16_t hour);

/**
 * Compute the auxiliary energy variable z_energy = energy + y_energy,
 * then clamp to [0, infinity) to enforce non-negativity.
 *
 * @param energy_in   Current energy state values
 * @param y_energy_in Dual variable for energy balance
 * @param hour        Number of hours to process
 */
void update_z_energy(float *energy_in, float *y_energy_in, uint16_t hour);

/**
 * Return the maximum of two floats.
 */
float max_f(float a, float b);

/**
 * Return the minimum of two floats.
 */
float min_f(float a, float b);

/**
 * Compute the L2 Euclidean norm of a vector.
 * @param vec   Pointer to vector data
 * @param size  Number of elements
 * @return      Euclidean norm (sqrt of sum of squares)
 */
float norm_vector(float *vec, uint16_t size);

/* --- Constants --- */
#define MAX_ITER 1000           ///< Maximum ADMM iterations
#define COMM_THRESHOLD 0.2f     ///< Communication threshold flag
#define STEPS_PER_DAY 96        ///< Number of time steps per day (15-min intervals)

/* --- External variables (global state) --- */
extern __IO uint16_t current_day;         ///< Current simulation day
extern __IO uint16_t current_step;        ///< Current step within day
extern __IO uint16_t duration;            ///< Remaining charging duration (hours)
extern __IO uint16_t period;              ///< Coordination period (hours)
extern __IO float energy_demand;          ///< Total energy required by EV (kWh)
extern __IO float power_max;              ///< Maximum charging power (kW)
extern __IO uint8_t plug_status;          ///< 1 if EV is plugged in, 0 otherwise
extern __IO uint8_t communicate_sign;     ///< Flag for communication status
extern __IO uint8_t first_duration;       ///< Flag for first optimization run

extern float upper_variable[MAX_HOUR];     ///< Upper bound variable for power constraint
extern float dual[MAX_HOUR];               ///< Dual variable (Lagrange multipliers)
extern float lower_variable[MAX_HOUR];     ///< Lower bound variable for power

#endif /* __OPTIMIZATION_H__ */