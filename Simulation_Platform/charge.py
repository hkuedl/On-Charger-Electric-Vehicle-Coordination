from gurobipy import *  
from pyomo.environ import *  
import numpy as np  
import os  
import csv  

# Global parameters for EV charging optimization
eta = 0.95      # Charging efficiency
alpha = 0.0016  # Battery degradation cost coefficient
betaa = 1.0     # Comfort penalty coefficient
delta = 0.25    # Time step duration (15 minutes)

class Charger():
    """
    EV Charger class for charging optimization and power control.
    """
    
    def __init__(self, ev_data, lmp_price_data, period, path) -> None:
        """
        Initialize the charger with EV data and pricing information.
        
        Args:
            ev_data: DataFrame containing EV arrival/departure data
            lmp_price_data: DataFrame containing locational marginal pricing data
            period: Optimization horizon period
            path: File path for logging results
        """
        self.ev_data = ev_data
        self.lmp_price_data = lmp_price_data
        self.period = period
        self.path = path
        
        # Initialize tracking variables
        self.record = 0      # Time record counter
        self.duration = 0    # Remaining charging duration
        self.plug = 0        # Plug-in status (0: unplugged, 1: plugged)
        self.cost = []       # List to track charging costs
        self.degration = []  # List to track battery degradation costs
        self.comfort = []    # List to track comfort penalty costs

    def file_path(self, directory):
        """
        Create CSV file for logging charging session data.
        
        Args:
            directory: Directory path where CSV file will be created
        """
        os.makedirs(directory, exist_ok=True)
        self.csv_file_path = os.path.join(directory, f"{self.path}.csv")
        with open(self.csv_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            # Write CSV header
            writer.writerow(['day', 'step', 'price', 'duration', 'power', 'energy', 'cost', 'degration', 'comfort'])

    def time_step(self, day, step):
        """
        Update the current time step in the simulation.
        
        Args:
            day: Current simulation day
            step: Current time step within the day
        """
        self.day = day
        self.step = step

    def load_data(self):
        """
        Load EV charging session data based on current time.
        Checks if an EV arrives at the current time step.
        """
        self.energy_demand = 0  # Energy required by EV (kWh)
        self.power_max = 0      # Maximum charging power limit (kW)
        
        # Search for EV arrival at current time
        for _, row in self.ev_data.iterrows():
            start_day = row['start day']
            start_step = row['start step']
            if self.day == start_day and self.step == start_step:
                leave_day = row['leave day']
                leave_step = row['leave step']
                # Calculate total duration in time steps (96 steps per day = 15min intervals)
                self.duration = int((leave_day - start_day) * 96 + (leave_step - start_step))
                self.energy_demand = row['energy demand']
                self.power_max = row['power limit']
                break
                
    def state_update(self):
        """
        Update the charger state and initialize optimization variables.
        """
        record = self.day * 96 + self.step  # Convert day/step to absolute time index
        if self.duration == 0:
            self.load_data()  # Check for new EV arrivals
            self.first_duration = True

        if self.duration > 0:
            # EV is connected - initialize charging optimization variables
            self.plug = 1
            self.power = np.zeros(self.duration)  # Charging power schedule
            
            # Algorithm variables
            self.upper_variable = np.zeros(self.duration)  # Upper-level variables
            self.dual = self.lmp_price_data['price'].iloc[record: record+self.duration].values.copy()
            self.power_old = np.zeros(self.duration)  # Previous iteration power values
            
            if hasattr(self, 'first_duration') and self.first_duration:
                self.lower_variable = np.zeros(self.duration)  # Lower-level variables
                self.first_duration = False
            else:
                # Shift lower variables for next time step
                self.lower_variable = self.lower_variable[1:]
        else:
            self.plug = 0  # No EV connected
        
        self.communicate_sign = 1  # Communication flag for distributed optimization

    def upper_variable_update(self, upper_variable, dual):
        """
        Update upper-level variables and dual prices from microgrid operator.
        
        Args:
            upper_variable: Upper-level decision variables from microgrid platform
            dual: Updated dual prices/multipliers
        """
        for t in range(min(self.duration, self.period)):
            self.upper_variable[t] = upper_variable[t]
            self.dual[t] = dual[t]

    def event_triggered_communicate(self):
        """
        Determine whether communication with microgrid platform is needed.
        """
        threshold = 0.2  # Communication threshold
        distance = np.linalg.norm(self.lower_variable - self.power)
        
        if distance > threshold:
            self.communicate_sign = 1  # Trigger communication
            # Update lower variables with current power values
            for t in range(self.duration):
                self.lower_variable[t] = self.power[t]
        else:
            self.communicate_sign = 0  # No communication needed

    def lower_variable_communicate(self):
        """
        Prepare lower-level variables for communication with microgrid platform.
        
        Returns:
            power: Power schedule for the optimization period
            communicate_sign: Flag indicating if communication is needed
        """
        power = np.zeros(self.period)
        
        for t in range(min(self.duration, self.period)):
            power[t] = self.lower_variable[t]
        
        return power, self.communicate_sign
      
    def lower_state_upload(self):
        """
        Upload current charger state information to microgrid platform.
        
        Returns:
            duration: Remaining charging duration
            energy_demand: Current energy demand
            power_max: Maximum power limit
        """
        return self.duration, self.energy_demand, self.power_max

    def state_transfer(self):
        """
        Execute one time step of charging and update system state and calculate actual power, energy transfer, and associated costs.
        """
        if self.plug == 1:
            # EV is connected - apply power and price constraints
            power = self.power[0]   
            power = max(0, min(power, self.power_max))  # Clip power to feasible range
            price = self.dual[0]
        else:
            # No EV connected
            price = 0
            power = 0

        # State transition: update energy demand based on charging
        energy_demand = self.energy_demand - eta * delta * power
        if energy_demand < 0:
            # Energy demand fully satisfied
            actual_demand = 0
            actual_power = power - (actual_demand - energy_demand) / (eta * delta)
        else:
            actual_demand = energy_demand
            actual_power = power
        
        # Calculate costs for current time step
        cost = actual_power * price * delta           # Electricity cost
        degration = alpha * (actual_power * delta) ** 2  # Battery degradation cost
        
        # Track costs
        self.cost.append(cost)
        self.degration.append(degration)
        
        # Comfort penalty (applied only at departure)
        if self.duration == 1:
            comfort = betaa * actual_demand  # Penalty for unmet energy demand
        else:
            comfort = 0
        self.comfort.append(comfort)
        
        # Update system state
        self.energy_demand = actual_demand
        if self.duration > 0:
            self.duration = self.duration - 1

        # Log results to CSV file
        with open(self.csv_file_path, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.day, self.step, price, self.duration,
                actual_power, actual_demand, cost, degration, comfort
            ])

    def get_cost(self):
        """
        Calculate total accumulated costs.
        
        Returns:
            Total electricity cost, degradation cost, and comfort penalty
        """
        return sum(self.cost), sum(self.degration), sum(self.comfort)

    def lower_level_optimization(self, rho):
        """
        Solve lower-level optimization problem using Gurobi.
        
        Args:
            rho: Penalty parameter for consensus constraints
            
        Returns:
            bool: True if optimization succeeded, False otherwise
        """
        model = ConcreteModel()
        
        # Define time sets
        model.T = Set(initialize=[i for i in range(self.duration)])          # Charging periods
        model.T_plus = Set(initialize=[i for i in range(self.duration + 1)])  # Energy states
        model.T_g = Set(initialize=[i for i in range(min(self.duration, self.period))])  # Coordination periods

        # Parameters
        model.upper_variable = Param(model.T, initialize={t: self.upper_variable[t] for t in model.T})
        model.dual = Param(model.T, initialize={t: self.dual[t] for t in model.T})
        model.power_old = Param(model.T, initialize={t: self.power_old[t] for t in model.T})
        model.power_max = Param(initialize=self.power_max)
        model.energy_demand = Param(initialize=self.energy_demand)
        
        # Decision variables
        model.P_e = Var(model.T, within=NonNegativeReals)      # Charging power
        model.E_e = Var(model.T_plus, within=NonNegativeReals) # Battery energy state
        model.E_e[0].fix(model.energy_demand)  # Initial energy demand

        def obj_rule(model):
            """
            Objective function: minimize total cost including penalty term
            """
            # Original cost: comfort penalty + electricity cost + degradation cost
            cost = (betaa * model.E_e[self.duration] + 
                   sum(model.dual[t] * model.P_e[t] * delta + 
                       alpha * (model.P_e[t] * delta) ** 2 for t in model.T))
            
            # penalty term for consensus
            penalty = (rho / 2 * 
                      sum((model.P_e[t] * delta - model.power_old[t] * delta - 
                           model.upper_variable[t] * delta) ** 2 for t in model.T_g))
            return cost + penalty

        def ev_power_max_rule(model, t):
            """Power limit constraint"""
            return model.P_e[t] <= model.power_max

        def ev_transfer_rule(model, t):
            """Energy balance constraint"""
            return model.E_e[t+1] == model.E_e[t] - eta * model.P_e[t] * delta

        # Define model components
        model.obj = Objective(rule=obj_rule, sense=minimize)
        model.ev_power_max_rule = Constraint(model.T, rule=ev_power_max_rule)
        model.ev_transfer_rule = Constraint(model.T, rule=ev_transfer_rule)
        
        # Solve optimization problem
        opt = SolverFactory('gurobi')
        opt.options['DisplayInterval'] = 1
        try:
            solution = opt.solve(model, warmstart=True)
            
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Lower warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False
            
            # Update power schedule with optimal solution
            self.power_old = self.power.copy()
            for t in model.T:
                self.power[t] = max(0.0, value(model.P_e[t]))
            
            return True
        
        except Exception as e:
            print(f"Lower error solving optimization problem: {e}")
            return False
      
    def lower_level_optimization_tiny(self, rho):
        """
        Solve lower-level optimization problem using TinyMPC.
        
        Args:
            rho: Penalty parameter for outer loop
        """
        
        def update_power_energy(hour, period, price, w_power, g_power, y_energy, z_energy, energy_demand, upper_variable, rho_inloop, rho_outloop):
            """
            Update power and energy variables using discrete Riccati equation for LQR
            
            Returns:
                power: Optimal power schedule
                energy: Corresponding energy trajectory
            """
            # LQR cost matrices
            Q_f = rho_inloop   # Final state cost
            Q = rho_inloop     # State cost
            R = np.full(hour, alpha * 2 + rho_inloop)  # Control cost
            
            # LQR cost vectors
            q_f = betaa + rho_inloop * (y_energy[hour] - z_energy[hour])
            q_k = rho_inloop * (y_energy[:-1] - z_energy[:-1])
            r_k = price + rho_inloop * (g_power - w_power)
            
            # Add coordination penalty terms
            for t in range(min(hour, period)):
                r_k[t] = r_k[t] - rho_outloop * upper_variable[t] * delta
                R[t] = R[t] + rho_outloop
            
            # System dynamics matrices
            A = 1.0      # Energy state transition
            B = -eta     # Power-to-energy coupling
            
            # Riccati equation backward pass
            P = np.zeros(hour+1)  # Cost-to-go value function
            p = np.zeros(hour+1)  # Cost-to-go linear term
            K = np.zeros(hour)    # Feedback gain
            d = np.zeros(hour)    # Feedforward term
            
            # Terminal conditions
            P[hour] = Q_f
            p[hour] = q_f
            
            # Backward recursion
            for t in reversed(range(hour)):
                P_next = P[t+1]
                p_next = p[t+1]
                
                # Handle numerical stability
                denom = R[t] + B**2 * P_next
                if abs(denom) < 1e-12:
                    denom = 1e-12 if denom >= 0 else -1e-12

                # Optimal control law coefficients
                K[t] = (A * B * P_next) / denom
                d[t] = (B * p_next + r_k[t]) / denom

                # Update cost-to-go function
                P[t] = Q + A**2 * P_next - (A * B * P_next)**2 / denom
                p[t] = q_k[t] + A * (p_next - B * P_next * d[t])
            
            # Forward pass: compute optimal trajectory
            power = np.zeros(hour)
            energy = np.zeros(hour+1)
            energy[0] = energy_demand
            
            for t in range(hour):
                power[t] = -K[t] * energy[t] - d[t]  # Optimal control law
                energy[t+1] = A * energy[t] + B * power[t]  # State evolution
                
            return power, energy

        def update_w_power(power, g_power, power_max):
            """
            Update power auxiliary variable with box constraints
            
            Args:
                power: Current power values
                g_power: Lagrange multipliers
                power_max: Maximum power limit
                
            Returns:
                w: Updated auxiliary variable
            """
            w = power + g_power
            w = np.minimum(w, power_max * delta)  # Upper bound
            w = np.maximum(w, 0)                  # Lower bound (non-negative)
            return w

        def update_z_energy(energy, y_energy):
            """
            Update energy auxiliary variable with non-negativity constraint
            
            Args:
                energy: Current energy values
                y_energy: Lagrange multipliers
                
            Returns:
                z: Updated auxiliary variable
            """
            z = energy + y_energy
            z = np.maximum(z, 0)  # Non-negative constraint
            return z

        # algorithm parameters
        rho_outloop = rho      # Outer loop penalty (coordination)
        rho_inloop = 0.05      # Inner loop penalty (constraint handling)
        mu = 10.0              # Residual balance parameter
        tau = 2.0              # Penalty update factor
        rho_min = 1e-3         # Minimum penalty parameter
        rho_max = 1e3          # Maximum penalty parameter
        primal_tol = 1e-3      # Primal residual tolerance
        dual_tol = 1e-3        # Dual residual tolerance
        max_iter = 1000        # Maximum iterations

        # Problem dimensions and data
        hour = self.duration
        period = self.period
        price = self.dual
        energy_demand = self.energy_demand
        power_max = self.power_max

        # Initialize primal variables
        power = np.zeros(hour)
        energy = np.zeros(hour+1)
        w_power = np.zeros(hour)     # Power auxiliary variables
        z_energy = np.zeros(hour+1)  # Energy auxiliary variables

        # Initialize dual variables (Lagrange multipliers)
        y_energy = np.zeros(hour+1)  # Energy constraint multipliers
        g_power = np.zeros(hour)     # Power constraint multipliers
        
        # Convergence tracking
        primal_residuals = []
        dual_residuals = []

        # main iteration loop
        for iteration in range(max_iter):
            # Store previous auxiliary variables for dual residual calculation
            w_power_old = w_power.copy()
            z_energy_old = z_energy.copy()

            # updates
            power, energy = update_power_energy(
                hour, period, price, w_power, g_power, y_energy, z_energy,
                energy_demand, self.upper_variable, rho_inloop, rho_outloop)
            
            w_power = update_w_power(power, g_power, power_max)
            z_energy = update_z_energy(energy, y_energy)
            
            # Dual variable updates
            g_power += power - w_power
            y_energy += energy - z_energy
            
            # Compute residuals for convergence check
            primal_res_vec = np.hstack([power - w_power, energy - z_energy])
            primal_residual = np.linalg.norm(primal_res_vec)
            
            dual_res_vec = np.hstack([power - w_power_old, energy - z_energy_old])
            dual_residual = rho_inloop * np.linalg.norm(dual_res_vec)

            primal_residuals.append(float(primal_residual))
            dual_residuals.append(float(dual_residual))
            
            # Adaptive penalty parameter update
            if iteration > 0 and iteration % 20 == 0:
                if primal_residual > mu * dual_residual:
                    rho_inloop = min(tau * rho_inloop, rho_max)
                elif dual_residual > mu * primal_residual:
                    rho_inloop = max(rho_inloop / tau, rho_min)
            
            # Check convergence
            if primal_residual < primal_tol and dual_residual < dual_tol:
                # Update power schedule and trigger communication check
                for t in range(self.duration):
                    self.power[t] = max(0.0, power[t] / delta)
                self.event_triggered_communicate()
                break
      
    def local_optimization(self):
        """
        Solve lower-level optimization problem using local optimization without coordination constraints.
        """
        model = ConcreteModel()
        
        # Time sets
        model.T = Set(initialize=[i for i in range(self.duration)])
        model.T_plus = Set(initialize=[i for i in range(self.duration + 1)])

        # Parameters
        model.dual = Param(model.T, initialize={t: self.dual[t] for t in model.T})
        model.power_old = Param(model.T, initialize={t: self.power_old[t] for t in model.T})
        model.power_max = Param(initialize=self.power_max)
        model.energy_demand = Param(initialize=self.energy_demand)
        
        # Variables
        model.P_e = Var(model.T, within=NonNegativeReals)
        model.E_e = Var(model.T_plus, within=NonNegativeReals)
        model.E_e[0].fix(model.energy_demand)

        def obj_rule(model):
            """
            Local objective: minimize individual EV charging cost.
            """
            cost = (betaa * model.E_e[self.duration] + 
                   sum(model.dual[t] * model.P_e[t] * delta + 
                       alpha * (model.P_e[t] * delta) ** 2 for t in model.T))
            return cost

        def ev_power_max_rule(model, t):
            """Power limit constraint"""
            return model.P_e[t] <= model.power_max

        def ev_transfer_rule(model, t):
            """Energy balance constraint"""
            return model.E_e[t+1] == model.E_e[t] - eta * model.P_e[t] * delta

        model.obj = Objective(rule=obj_rule, sense=minimize)
        model.ev_power_max_rule = Constraint(model.T, rule=ev_power_max_rule)
        model.ev_transfer_rule = Constraint(model.T, rule=ev_transfer_rule)
        
        opt = SolverFactory('gurobi')
        opt.options['Threads'] = 32
        try:
            solution = opt.solve(model, warmstart=True)
            if solution.solver.termination_condition != TerminationCondition.optimal:
                print(f"Lower warning: Solver did not find optimal solution. Status: {solution.solver.termination_condition}")
                return False
            
            self.power_old = self.power.copy()
            for t in model.T:
                self.power[t] = max(0.0, value(model.P_e[t]))
            
            return True
        
        except Exception as e:
            print(f"Lower error solving optimization problem: {e}")
            return False
          
    def global_optimization(self):
        """
        Charge at minimum of maximum feasible power from microgrid platform. 
        """
        self.power[0] = min(min(self.energy_demand / (eta * delta), self.power_max), 
                           self.upper_variable[0])
      
    def plug_and_play_optimization(self):
        """
        Charge as fast as possible without coordination.
        """
        self.power[0] = min(self.energy_demand / (eta * delta), self.power_max)

    def centralized_optimization(self, power):
        """
        Use power command from microgrid platform.
        
        Args:
            power: Power command from microgrid platform
        """
        self.power[0] = power