import numpy as np  
import os  
import csv  
from gurobipy import *  
from pyomo.environ import *  

# System parameters for microgrid optimization
M = 1000           # Big-M constant for binary constraints
P_max = 1500       # Maximum grid power import/export (kW)
es_capacity = 200  # Energy storage capacity (kWh)
es_power_max = 100 # Maximum energy storage charging/discharging power (kW)
eta = 0.95         # Energy storage round-trip efficiency
delta = 0.25       # Time step duration (15 minutes = 0.25 hours)

# Cost coefficients for EV charging optimization
alpha = 0.0016     # Battery degradation cost coefficient
betaa = 1.0        # Comfort penalty coefficient for unmet energy demand

class Microgrid():
    """
    Microgrid platform class that handles upper-level optimization for distributed energy resources.
    """

    def __init__(self, period, lmp_price_data, base_data, pv_data) -> None:
        """
        Initialize the microgrid system with data and optimization parameters
        
        Args:
            period: Optimization horizon period (number of time steps)
            lmp_price_data: DataFrame containing locational marginal pricing data
            base_data: DataFrame containing base load demand forecast
            pv_data: DataFrame containing PV generation forecast
        """
        self.period = period            # Optimization horizon length
        self.lmp_price_data = lmp_price_data  # Grid pricing data
        self.base_data = base_data      # Base load forecast
        self.pv_data = pv_data         # PV generation forecast

        # Initialize system state variables
        self.day = 0                   # Current simulation day
        self.step = 0                  # Current time step within day
        self.es_energy = 0             # Current energy storage state of charge
        self.revenue = []              # List tracking revenue from EV charging
        self.cost = []                 # List tracking grid electricity costs

    def file_path(self, directory):
        """
        Create CSV file for logging microgrid operational data
        
        Args:
            directory: Directory path where log file will be created
        """
        os.makedirs(directory, exist_ok=True)
        self.csv_file_path = os.path.join(directory, 'grid.csv')
        with open(self.csv_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            # Write CSV header with all tracked variables
            writer.writerow(['day', 'step', 'grid_price', 'tou_price', 'ev_power', 
                           'base_load', 'es_power', 'grid_power', 'pv_acc_power', 
                           'pv_power', 'revenue', 'cost'])

    def time_step(self, day, step):
        """
        Update the current time step in the microgrid simulation.
        
        Args:
            day: Current simulation day
            step: Current time step within the day (0-95 for 15-min intervals)
        """
        self.day = day
        self.step = step

    def load_data(self):
        """
        Load data for the current optimization horizon.
        
        Returns:
            base: Base load demand for optimization period
            pv: PV generation for optimization period  
            lmp_price: Grid electricity price for optimization period
        """
        record = self.day * 96 + self.step  # Convert to absolute time index
        
        # Extract data for optimization horizon
        base = self.base_data['power'].iloc[record: record+self.period].values.copy()
        pv = self.pv_data['power'].iloc[record: record+self.period].values.copy()
        lmp_price = self.lmp_price_data['price'].iloc[record: record+self.period].values.copy()

        return base, pv, lmp_price
      
    def state_update(self, number):
        """
        Update microgrid state and initialize optimization variables for current time step.
        
        Args:
            number: Number of active EV chargers currently connected
        """
        # Load data for optimization horizon
        base, pv, lmp_price = self.load_data()
        self.base_load = base       # Base load demand forecast
        self.pv_output = pv         # PV generation forecast
        self.number = number        # Number of active EV chargers
        self.grid_price = lmp_price.copy()  # Grid electricity prices

        # Initialize algorithm variables
        self.lower_variable = np.zeros(self.period)  # Lower-level power variables from EVs
        self.ev_power_old = np.zeros(self.period)    # Previous iteration EV power
        self.ev_power = np.zeros(self.period)        # Current EV power schedule

        # Initialize upper-level decision variables
        self.upper_variable = np.zeros(self.period)

        # Update dual variables (prices) with time-shifting
        if hasattr(self, 'dual') and self.dual is not None:
            # Shift dual variables forward in time
            self.dual[:-1] = self.dual[1:]
            self.dual[-1] = lmp_price[-1].copy()  # Add new price at end
        else:
            # Initialize dual variables with grid prices
            self.dual = lmp_price.copy()

        # Initialize power outputs for current time step
        self.grid_power_0 = 0    # Grid power import/export
        self.pv_acc_0 = 0        # Accommodated PV power  
        self.es_power_0 = 0      # Energy storage power (positive = charging)

    def lower_variable_update(self, all_lower_variable):
        """
        Update lower-level variables received from EV chargers.
        
        Args:
            all_lower_variable: Aggregated power schedule from all EV chargers
        """
        for t in range(self.period):
            # Average power per EV charger
            self.lower_variable[t] = all_lower_variable[t] / self.number
          
    def local_result_update(self, all_lower_variable):
        """
        Update EV power variables for local optimization results.
        
        Args:
            all_lower_variable: Aggregated EV power schedule
        """
        for t in range(self.period):
            self.ev_power[t] = all_lower_variable[t] / self.number

    def local_state_update(self, duration, energy_demand, ev_power_max):
        """
        Update EV state information for centralized optimization.
        
        Args:
            duration: Array of remaining charging durations for each EV
            energy_demand: Array of energy demands for each EV
            ev_power_max: Array of maximum power limits for each EV
        """
        self.duration = duration
        self.energy_demand = energy_demand
        self.ev_power_max = ev_power_max

    def upper_variable_communicate(self):
        """
        Prepare upper-level variables and dual prices for communication to EV chargers.
        
        Returns:
            upper_variable: Upper-level decision variables
            dual: Updated dual prices/multipliers
        """
        for t in range(self.period):
            # Calculate consensus residual as upper variable
            self.upper_variable[t] = self.ev_power[t] - self.lower_variable[t]
            # Ensure dual prices are non-negative
            self.dual[t] = max(0, self.dual[t])
   
        return self.upper_variable, self.dual

    def upper_result_update(self):
        """
        Get individual EV power commands for centralized optimization
        
        Returns:
            ev_power_n: Array of power commands for each EV
        """
        return self.ev_power_n

    def upper_level_update(self, rho):
        """
        Solve upper-level microgrid optimization problem with EV coordination.
        
        Args:
            rho: Penalty parameter for consensus constraints
            
        Returns:
            bool: True if optimization succeeded, False otherwise
        """
        model = ConcreteModel()
        model.T = Set(initialize=[i for i in range(self.period)])  # Time periods

        # Parameters from data and algorithm
        model.price_g = Param(model.T, initialize=self.grid_price)
        model.P_base = Param(model.T, initialize=self.base_load)
        model.P_pv = Param(model.T, initialize=self.pv_output)
        model.P_max = Param(initialize=P_max)
        model.number = Param(initialize=self.number)
        model.lower_variable = Param(model.T, initialize=self.lower_variable)
        model.dual = Param(model.T, initialize=self.dual)

        # Decision variables
        model.P_g = Var(model.T, within=NonNegativeReals)                    # Grid power
        model.P_e = Var(model.T, within=NonNegativeReals)                    # EV charging power per charger
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)               # Accommodated PV power
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))   # ES charging
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))  # ES discharging
        model.U_es = Var(model.T, within=Binary)                            # ES charging/discharging mode
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, es_capacity))       # ES energy level

        def obj_rule(model):
            """
            Objective function: minimize total microgrid cost including penalty
            """
            # Grid electricity cost
            cost_grid = sum(model.price_g[t] * model.P_g[t] * delta for t in model.T)
            # Revenue from EV charging (negative cost)
            revenue_ev = sum(-model.dual[t] * (model.P_e[t] * model.number) * delta for t in model.T)
            # Penalty for consensus between upper and lower levels
            penalty = sum(model.number * (rho / 2) * (model.P_e[t] - model.lower_variable[t]) ** 2 for t in model.T)
            return cost_grid + revenue_ev + penalty
          
        def all_active_power_rule(model, t):
            """Power balance constraint for microgrid bus"""
            return (model.P_g[t] + model.P_pv_acc[t] == 
                   model.P_base[t] + model.P_es_ch[t] - model.P_es_dis[t] + model.P_e[t] * model.number)
            
        def pv_accomodation_rule(model, t):
            """PV accommodation cannot exceed available generation"""
            return model.P_pv_acc[t] <= model.P_pv[t]
          
        def storage_transfer_rule(model, t):
            """Energy storage state transition"""
            if t == 0:
                es_energy = self.es_energy  # Use current storage level
            else:
                es_energy = model.E_es[t-1]  # Use previous time step
            
            return (model.E_es[t] == es_energy + eta * delta * model.P_es_ch[t] - 
                   (1 / eta) * delta * model.P_es_dis[t])

        def charging_limit_rule(model, t):
            """Energy storage can only charge when binary variable is 1"""
            return model.P_es_ch[t] <= M * model.U_es[t]

        def discharging_limit_rule(model, t):
            """Energy storage can only discharge when binary variable is 0"""  
            return model.P_es_dis[t] <= M * (1 - model.U_es[t])

        def active_power_max_rule(model, t):
            """Grid power import limit"""
            return model.P_g[t] <= model.P_max

        # Define model components
        model.objective = Objective(rule=obj_rule, sense=minimize)
        model.active_power_max = Constraint(model.T, rule=active_power_max_rule)
        model.all_active_power = Constraint(model.T, rule=all_active_power_rule)
        model.pv_accomodation = Constraint(model.T, rule=pv_accomodation_rule)
        model.storage_transfer = Constraint(model.T, rule=storage_transfer_rule)
        model.charging_limit = Constraint(model.T, rule=charging_limit_rule)
        model.discharging_limit = Constraint(model.T, rule=discharging_limit_rule)

        # Solve optimization problem
        opt = SolverFactory('gurobi')

        try:
            solution = opt.solve(model, warmstart=True)
           
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Upper warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False
            
            # Extract optimal solution for EV power schedule
            self.ev_power_old = self.ev_power.copy()
            for t in model.T:
                self.ev_power[t] = value(model.P_e[t])

            # Extract power outputs for current time step
            self.grid_power_0 = value(model.P_g[0])
            self.pv_acc_0 = value(model.P_pv_acc[0])
            self.pv_0 = value(model.P_pv[0])
            self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])
            
            return True
              
        except Exception as e:
            print(f"Upper error solving optimization problem: {e}")
            return False
              
    def upper_level_update_noev(self):
        """
        Solve microgrid optimization without EV coordination.
        
        Returns:
            bool: True if optimization succeeded, False otherwise
        """
        model = ConcreteModel()
        model.T = Set(initialize=[i for i in range(self.period)])

        # Parameters (no EV-related parameters)
        model.price_g = Param(model.T, initialize=self.grid_price)
        model.P_base = Param(model.T, initialize=self.base_load)
        model.P_pv = Param(model.T, initialize=self.pv_output)
        model.P_max = Param(initialize=P_max)

        # Decision variables (no EV variables)
        model.P_g = Var(model.T, within=NonNegativeReals)
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.U_es = Var(model.T, within=Binary)
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, es_capacity))

        def obj_rule(model):
            """Objective: minimize only grid electricity cost"""
            cost_grid = sum(model.price_g[t] * model.P_g[t] * delta for t in model.T)
            return cost_grid
          
        def all_active_power_rule(model, t):
            """Power balance without EV load"""
            return (model.P_g[t] + model.P_pv_acc[t] == 
                   model.P_base[t] + model.P_es_ch[t] - model.P_es_dis[t])
            
        def pv_accomodation_rule(model, t):
            """PV accommodation constraint"""
            return model.P_pv_acc[t] <= model.P_pv[t]
          
        def storage_transfer_rule(model, t):
            """Energy storage dynamics"""
            if t == 0:
                es_energy = self.es_energy
            else:
                es_energy = model.E_es[t-1]
            
            return (model.E_es[t] == es_energy + eta * delta * model.P_es_ch[t] - 
                   (1 / eta) * delta * model.P_es_dis[t])

        def charging_limit_rule(model, t):
            """Storage charging limit"""
            return model.P_es_ch[t] <= M * model.U_es[t]

        def discharging_limit_rule(model, t):
            """Storage discharging limit"""
            return model.P_es_dis[t] <= M * (1 - model.U_es[t])

        def active_power_max_rule(model, t):
            """Grid power limit"""
            return model.P_g[t] <= model.P_max

        # Define model components
        model.objective = Objective(rule=obj_rule, sense=minimize)
        model.active_power_max = Constraint(model.T, rule=active_power_max_rule)
        model.all_active_power = Constraint(model.T, rule=all_active_power_rule)
        model.pv_accomodation = Constraint(model.T, rule=pv_accomodation_rule)
        model.storage_transfer = Constraint(model.T, rule=storage_transfer_rule)
        model.charging_limit = Constraint(model.T, rule=charging_limit_rule)
        model.discharging_limit = Constraint(model.T, rule=discharging_limit_rule)

        opt = SolverFactory('gurobi')
        opt.options['Threads'] = 32
        try:
            solution = opt.solve(model, warmstart=True)
           
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Upper warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False

            # Extract power outputs for current time step
            self.grid_power_0 = value(model.P_g[0])
            self.pv_acc_0 = value(model.P_pv_acc[0])
            self.pv_0 = value(model.P_pv[0])
            self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])
            
            return True
              
        except Exception as e:
            print(f"Upper error solving optimization problem: {e}")
            return False

    def global_optimization(self):
        """
        Solve global microgrid optimization with EV power allocation.
        
        Returns:
            bool: True if optimization succeeded, False otherwise
        """
        model = ConcreteModel()
        model.T = Set(initialize=[i for i in range(self.period)])

        # Parameters
        model.price_g = Param(model.T, initialize=self.grid_price)
        model.P_base = Param(model.T, initialize=self.base_load)
        model.P_pv = Param(model.T, initialize=self.pv_output)
        model.P_max = Param(initialize=P_max)
        model.number = Param(initialize=self.number)
        model.lower_variable = Param(model.T, initialize=self.lower_variable)
        model.dual = Param(model.T, initialize=self.dual)

        # Decision variables
        model.P_g = Var(model.T, within=NonNegativeReals)
        model.P_e = Var(model.T, within=NonNegativeReals)
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.U_es = Var(model.T, within=Binary)
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, es_capacity))
        model.P_e_avg = Var()  # Average EV charging power

        def obj_rule(model):
            """
            Multi-objective function: minimize cost, maximize renewables, balance load
            """
            # Grid electricity cost
            cost_grid = sum(model.price_g[t] * model.P_g[t] * delta for t in model.T)
            # Renewable energy incentive (negative cost)
            renewable_penalty = -sum(10 * model.P_pv_acc[t] * delta for t in model.T)
            # Load balancing penalty (minimize charging power variation)
            avg_penalty = sum(0.5 * (model.P_e[t] * delta - model.P_e_avg * delta)**2 for t in model.T)
            return cost_grid + renewable_penalty + avg_penalty
          
        def all_active_power_rule(model, t):
            """Power balance constraint"""
            return (model.P_g[t] + model.P_pv_acc[t] == 
                   model.P_base[t] + model.P_es_ch[t] - model.P_es_dis[t] + model.P_e[t] * model.number)
            
        def pv_accomodation_rule(model, t):
            """PV accommodation constraint"""
            return model.P_pv_acc[t] <= model.P_pv[t]
          
        def storage_transfer_rule(model, t):
            """Energy storage dynamics"""
            if t == 0:
                es_energy = self.es_energy
            else:
                es_energy = model.E_es[t-1]
            
            return (model.E_es[t] == es_energy + eta * delta * model.P_es_ch[t] - 
                   (1 / eta) * delta * model.P_es_dis[t])

        def charging_limit_rule(model, t):
            """Storage charging limit"""
            return model.P_es_ch[t] <= M * model.U_es[t]

        def discharging_limit_rule(model, t):
            """Storage discharging limit"""
            return model.P_es_dis[t] <= M * (1 - model.U_es[t])

        def active_power_max_rule(model, t):
            """Grid power limit"""
            return model.P_g[t] <= model.P_max

        def avg_power_rule(model):
            """Define average EV charging power"""
            return model.P_e_avg * self.period == sum(model.P_e[t] for t in model.T)
          
        def min_power_rule(model):
            """
            Minimum energy delivery constraint based on available capacity
            Ensures adequate charging while respecting system limits
            """
            # Available energy capacity over optimization horizon
            capacity = sum(model.P_max + model.P_pv[t] - model.P_base[t] for t in model.T)
            # Minimum required energy (8 kW baseline per EV)
            threshold = 8 * model.number * self.period

            if capacity >= threshold:
                return sum(model.P_e[t] * model.number for t in model.T) >= threshold
            else:
                return sum(model.P_e[t] * model.number for t in model.T) >= capacity
          
        # Define model components
        model.min_power = Constraint(rule=min_power_rule)
        model.avg_power_constraint = Constraint(rule=avg_power_rule)
        model.objective = Objective(rule=obj_rule, sense=minimize)
        model.active_power_max = Constraint(model.T, rule=active_power_max_rule)
        model.all_active_power = Constraint(model.T, rule=all_active_power_rule)
        model.pv_accomodation = Constraint(model.T, rule=pv_accomodation_rule)
        model.storage_transfer = Constraint(model.T, rule=storage_transfer_rule)
        model.charging_limit = Constraint(model.T, rule=charging_limit_rule)
        model.discharging_limit = Constraint(model.T, rule=discharging_limit_rule)

        opt = SolverFactory('gurobi')

        try:
            solution = opt.solve(model, warmstart=True)
           
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Upper warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False
            
            # Extract optimal solution
            self.ev_power_old = self.ev_power.copy()
            for t in model.T:
                self.ev_power[t] = value(model.P_e[t])
                
            self.grid_power_0 = value(model.P_g[0])
            # Adjust PV accommodation for EV charging
            self.pv_acc_0 = value(model.P_pv_acc[0]) - value(model.P_e[0]) * self.number
            self.pv_0 = value(model.P_pv[0])
            self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])
            
            return True
              
        except Exception as e:
            print(f"Upper error solving optimization problem: {e}")
            return False
      
    def centralized_optimization(self):
        """
        Solve centralized microgrid optimization with individual EV modeling
        This provides the optimal benchmark solution with full information
        
        Returns:
            bool: True if optimization succeeded, False otherwise
        """
        model = ConcreteModel()
        model.T = Set(initialize=[i for i in range(self.period)])        # Time periods
        model.N = Set(initialize=[i for i in range(self.number)])        # Individual EVs

        # Parameters
        model.price_g = Param(model.T, initialize=self.grid_price)
        model.P_base = Param(model.T, initialize=self.base_load)
        model.P_pv = Param(model.T, initialize=self.pv_output)
        model.P_max = Param(initialize=P_max)  
        model.number = Param(initialize=self.number)
        model.energy_demand = Param(model.N, initialize=self.energy_demand)

        # Decision variables
        model.P_g = Var(model.T, within=NonNegativeReals)                 # Grid power
        model.P_e = Var(model.N, model.T, within=NonNegativeReals)       # Individual EV power
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)           # PV accommodation
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, es_power_max))
        model.U_es = Var(model.T, within=Binary)
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, es_capacity))
        model.E_ev = Var(model.N, within=NonNegativeReals)               # Unmet energy demand per EV
          
        def obj_rule(model):
            """
            Total system cost: grid cost + EV charging cost (degradation + comfort)
            """
            # Grid electricity cost
            cost_grid = sum(model.price_g[t] * model.P_g[t] * delta for t in model.T)
            # EV charging costs (degradation + comfort penalty for unmet demand)
            cost_ev = sum(betaa * model.E_ev[n] + 
                         sum(alpha * (model.P_e[n, t] * delta)** 2 for t in model.T) 
                         for n in model.N)
            return cost_grid + cost_ev
          
        def all_active_power_rule(model, t):
            """Power balance including all individual EVs"""
            return (model.P_g[t] + model.P_pv_acc[t] == 
                   model.P_base[t] + model.P_es_ch[t] - model.P_es_dis[t] + 
                   sum(model.P_e[n, t] for n in model.N))
            
        def pv_accomodation_rule(model, t):
            """PV accommodation constraint"""
            return model.P_pv_acc[t] <= model.P_pv[t]
          
        def storage_transfer_rule(model, t):
            """Energy storage dynamics"""
            if t == 0:
                es_energy = self.es_energy
            else:
                es_energy = model.E_es[t-1]
            
            return (model.E_es[t] == es_energy + eta * delta * model.P_es_ch[t] - 
                   (1 / eta) * delta * model.P_es_dis[t])

        def charging_limit_rule(model, t):
            """Storage charging limit"""
            return model.P_es_ch[t] <= M * model.U_es[t]

        def discharging_limit_rule(model, t):
            """Storage discharging limit"""
            return model.P_es_dis[t] <= M * (1 - model.U_es[t])

        def active_power_max_rule(model, t):
            """Grid power limit"""
            return model.P_g[t] <= model.P_max

        def ev_power_max_rule(model, n, t):
            """
            Individual EV power limits based on charging duration
            Power is zero if EV has already departed
            """
            if t < self.duration[n]:
                return model.P_e[n, t] <= self.ev_power_max[n]
            else:
                return model.P_e[n, t] == 0

        def ev_transfer_rule(model, n):
            """
            Energy balance for each EV: unmet demand = initial demand - charged energy
            """
            return (model.E_ev[n] == self.energy_demand[n] - 
                   sum(eta * model.P_e[n, t] * delta for t in model.T))
          
        # Define model components
        model.objective = Objective(rule=obj_rule, sense=minimize)
        model.active_power_max = Constraint(model.T, rule=active_power_max_rule)
        model.all_active_power = Constraint(model.T, rule=all_active_power_rule)
        model.pv_accomodation = Constraint(model.T, rule=pv_accomodation_rule)
        model.storage_transfer = Constraint(model.T, rule=storage_transfer_rule)
        model.charging_limit = Constraint(model.T, rule=charging_limit_rule)
        model.discharging_limit = Constraint(model.T, rule=discharging_limit_rule)
        model.ev_power_max = Constraint(model.N, model.T, rule=ev_power_max_rule)
        model.ev_transfer = Constraint(model.N, rule=ev_transfer_rule)

        opt = SolverFactory('gurobi')

        try:
            solution = opt.solve(model, warmstart=True)
           
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Upper warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False
                
            # Extract individual EV power commands and average
            self.ev_power_n = np.zeros(self.number)  # Individual EV powers
            self.ev_power_old = self.ev_power.copy()
            
            for t in model.T:
                # Calculate average EV power across all vehicles
                self.ev_power[t] = sum(value(model.P_e[n, t]) for n in model.N) / self.number
            
            for n in model.N:
                # Store individual power commands for current time step
                self.ev_power_n[n] = value(model.P_e[n, 0])

            # Extract microgrid power outputs
            self.grid_power_0 = value(model.P_g[0])
            self.pv_acc_0 = value(model.P_pv_acc[0])
            self.pv_0 = value(model.P_pv[0])
            self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])
            
            return True
              
        except Exception as e:
            print(f"Upper error solving optimization problem: {e}")
            return False
          
    def dual_update(self, rho):
        """
        Update dual variables (prices) in algorithm based on consensus violation
        
        Args:
            rho: Penalty parameter for dual update
        """
        for t in range(self.period):
            # Calculate primal residual (consensus violation)
            residual = (self.lower_variable[t] - self.ev_power[t]) * delta
            # Update dual variable with penalty
            self.dual[t] += rho * residual
            # Optional: enforce non-negative prices
            # self.dual[t] = max(0, self.dual[t])

    def compute_residual(self, rho):
        """
        Compute algorithm convergence residuals
        
        Args:
            rho: Penalty parameter
            
        Returns:
            primal_residual: L2 norm of primal feasibility violation
            dual_residual: L2 norm of dual feasibility violation
        """
        primal_residual = 0.0
        dual_residual = 0.0
        
        for t in range(self.period):
            # Primal residual: consensus constraint violation
            primal_residual += (self.lower_variable[t] * delta - self.ev_power[t] * delta) ** 2
            # Dual residual: change in upper-level variables
            dual_residual += (rho * (self.ev_power[t] * delta - self.ev_power_old[t] * delta)) ** 2

        return np.sqrt(primal_residual), np.sqrt(dual_residual)

    def state_transfer(self):
        """
        Execute one time step of microgrid operation and update system state
        Updates energy storage state and logs operational data
        """
        # Update energy storage state of charge
        if self.es_power_0 >= 0:
            # Charging: apply efficiency
            self.es_energy = max(0, self.es_energy + eta * self.es_power_0 * delta)
        else:
            # Discharging: apply efficiency loss
            self.es_energy = max(0, self.es_energy + (1 / eta) * self.es_power_0 * delta)

        # Calculate financial flows
        revenue = self.dual[0] * self.ev_power[0] * self.number * delta    # Revenue from EV charging
        cost = self.grid_price[0] * self.grid_power_0 * delta             # Cost of grid electricity

        # Track cumulative costs
        self.revenue.append(revenue)
        self.cost.append(cost)

        # Log operational data to CSV file
        with open(self.csv_file_path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.day, self.step,                # Time identifiers
                self.grid_price[0], self.dual[0],   # Grid price and EV charging price
                self.ev_power[0] * self.number,     # Total EV power
                self.base_load[0],                  # Base load demand
                self.es_power_0,                    # Energy storage power
                self.grid_power_0,                  # Grid import power
                self.pv_acc_0,                     # Accommodated PV power
                self.pv_output[0],                 # Available PV power
                revenue, cost                       # Financial flows
            ])

    def state_transfer_noev(self):
        """
        Execute microgrid operation without EV coordination (baseline scenario)
        Used for comparison studies
        """
        # Update energy storage state
        if self.es_power_0 >= 0:
            self.es_energy = max(0, self.es_energy + eta * self.es_power_0 * delta)
        else:
            self.es_energy = max(0, self.es_energy + (1 / eta) * self.es_power_0 * delta)

        # Calculate costs (no EV revenue in baseline)
        revenue = self.dual[0] * self.ev_power[0] * self.number * delta
        cost = self.grid_price[0] * self.grid_power_0 * delta
        
        # Calculate actual PV accommodation and grid power without optimization results
        pv_acc_0 = min(self.base_load[0] + self.es_power_0 + self.ev_power[0] * self.number, self.pv_output[0])
        grid_power_0 = self.base_load[0] + self.es_power_0 + self.ev_power[0] * self.number - pv_acc_0
        
        self.revenue.append(revenue)
        self.cost.append(cost)

        # Log baseline operational data
        with open(self.csv_file_path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.day, self.step,
                self.grid_price[0], self.dual[0],
                self.ev_power[0] * self.number,
                self.base_load[0],
                self.es_power_0,
                grid_power_0,                       # Calculated grid power
                pv_acc_0,                          # Calculated PV accommodation
                self.pv_output[0],
                revenue, cost
            ])

    def get_cost(self):
        """
        Get total accumulated financial flows for the microgrid
        
        Returns:
            Total revenue from EV charging and total cost of grid electricity
        """
        return sum(self.revenue), sum(self.cost)