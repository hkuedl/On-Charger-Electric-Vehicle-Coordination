import os

class System():
    """
    System class serves as the simulation, coordinating interactions between the microgrid and chargerss.
    """

    def __init__(self, micro_grid, chargers) -> None:
        """
        Initialize the system with microgrid and chargerss.
        
        Args:
            micro_grid: Microgrid platform (upper-level)
            chargers: Chargers (lower-level)
        """
        self.micro_grid = micro_grid
        self.chargers = chargers 

    def solution_distributed(self):
        """
        Execute distributed optimization solution using Gurobi solver.
        """
        # Create output directory for convergence analysis
        output_dir = "convergence_plots"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Algorithm parameters
        rho_init = 0.1      # Initial penalty parameter
        max_iter = 50       # Maximum iterations per time step
        primal_tol = 1e-1   # Primal residual convergence tolerance
        dual_tol = 1e-1     # Dual residual convergence tolerance
        mu = 10.0           # Residual balance parameter for penalty adjustment
        tau = 2             # Penalty adjustment factor
        rho_min = 1e-3      # Minimum penalty parameter
        rho_max = 1e3       # Maximum penalty parameter
        check_interval = 50 # Interval for penalty parameter adjustment
        
        # Communication tracking variables
        sum_number = 0      # Total number of EV charging sessions
        sum_commu = 0       # Total communication events
        
        # Set up logging directories
        self.micro_grid.file_path(directory='results/microgrid_distributed/')
        self.chargers.file_path(directory='results/record_distributed/')
        
        # Main simulation loop: 363 days, 96 time steps per day (15-min intervals)
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                # Update time step for all system components
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                
                # Check number of active EV chargers
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                # Update microgrid state
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)
                
                if number == 0:
                    # No EVs charging: solve microgrid optimization without EV coordination
                    self.micro_grid.upper_level_update_noev()
                else:
                    # EVs present: execute distributed optimization
                    primal_residuals = []  # Track primal residual convergence
                    dual_residuals = []    # Track dual residual convergence
                   
                    rho = rho_init  # Initialize penalty parameter
                    
                    # Initialize upper-level variables and dual prices
                    upper_variables, dual = self.micro_grid.upper_variable_communicate()
                    self.chargers.get_upper_variable(upper_variables, dual)
                    
                    # Iteration loop
                    for i in range(max_iter):
                        print(f"iteration {i} is conducting")
             
                        # Step 1: Lower-level optimization (EV chargers)
                        self.chargers.lower_level_optimization(rho)
                        
                        # Collect lower-level results and communication statistics
                        power, commu, numb = self.chargers.get_lower_variable()
                        sum_commu += commu  # Track communication events
                        sum_number += numb  # Track active chargers
                        print(f'communicate: {commu}, number of EVs: {numb}')
                        
                        # Update microgrid with aggregated EV power schedule
                        self.micro_grid.lower_variable_update(power)

                        # Step 2: Upper-level optimization (microgrid)
                        self.micro_grid.upper_level_update(rho)
                        
                        # Step 3: Dual variable update
                        self.micro_grid.dual_update(rho)

                        # Communicate updated variables back to chargers
                        upper_variables, dual = self.micro_grid.upper_variable_communicate()
                        self.chargers.get_upper_variable(upper_variables, dual)
                        
                        # Step 4: Check convergence
                        primal_residual, dual_residual = self.micro_grid.compute_residual(rho)
                        
                        print(f"primal residual is {primal_residual:.2e} and dual residual is {dual_residual:.2e}")
                        primal_residuals.append(primal_residual)
                        dual_residuals.append(dual_residual)
                        
                        # Check convergence criteria
                        if primal_residual < primal_tol and dual_residual < dual_tol:
                            break

                        # Step 5: Adaptive penalty parameter adjustment
                        if i > 0 and i % check_interval == 0:
                            if primal_residual > mu * dual_residual:
                                # Increase penalty to improve primal feasibility
                                old_rho = rho
                                rho = min(tau * rho, rho_max)
                                print(f"rho increase from {old_rho} to {rho}")
                            elif dual_residual > mu * primal_residual:
                                # Decrease penalty to improve dual feasibility
                                old_rho = rho
                                rho = max(rho / tau, rho_min)
                                print(f"rho decrease from {old_rho} to {rho}")
      
                # Execute one time step of actual charging and microgrid operation
                self.chargers.state_transfer()
                self.micro_grid.state_transfer()

        # Print final cost summary for all chargers
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        
        # Print microgrid financial summary
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")

    def solution_tiny(self):
        """
        Execute distributed optimization using TinyMPC.
        """
        # Algorithm parameters
        rho_init = 0.1
        max_iter = 50
        primal_tol = 1e-1
        dual_tol = 1e-1
        mu = 10.0
        tau = 2
        rho_min = 1e-3
        rho_max = 1e3
        check_interval = 50
        
        # Communication tracking
        sum_number = 0
        sum_commu = 0
        
        # Set up logging for tiny implementation
        self.micro_grid.file_path(directory='results/microgrid_tiny/')
        self.chargers.file_path(directory='results/record_tiny/')
    
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)
                
                if number == 0:
                    # No coordination needed
                    self.micro_grid.upper_level_update_noev()
                else:
                    # Riccati-based lower-level solver
                    primal_residuals = []
                    dual_residuals = []
                   
                    rho = rho_init
                    upper_variables, dual = self.micro_grid.upper_variable_communicate()
                    self.chargers.get_upper_variable(upper_variables, dual)
                    
                    for i in range(max_iter):
                        # Use tinyMPC method instead of Gurobi
                        self.chargers.lower_level_optimization_tiny(rho)
                        
                        power, commu, numb = self.chargers.get_lower_variable()
                        sum_commu += commu
                        sum_number += numb
                        print(f'communicate: {commu}, number of EVs: {numb}')
                        
                        self.micro_grid.lower_variable_update(power)
                        self.micro_grid.upper_level_update(rho)
                        self.micro_grid.dual_update(rho)

                        upper_variables, dual = self.micro_grid.upper_variable_communicate()
                        self.chargers.get_upper_variable(upper_variables, dual)
                        
                        primal_residual, dual_residual = self.micro_grid.compute_residual(rho)
                        print(f"primal residual is {primal_residual:.2e} and dual residual is {dual_residual:.2e}")
                        primal_residuals.append(primal_residual)
                        dual_residuals.append(dual_residual)
                        
                        if primal_residual < primal_tol and dual_residual < dual_tol:
                            break

                        # Adaptive penalty adjustment
                        if i > 0 and i % check_interval == 0:
                            if primal_residual > mu * dual_residual:
                                old_rho = rho
                                rho = min(tau * rho, rho_max)
                                print(f"rho increase from {old_rho} to {rho}")
                            elif dual_residual > mu * primal_residual:
                                old_rho = rho
                                rho = max(rho / tau, rho_min)
                                print(f"rho decrease from {old_rho} to {rho}")

                self.chargers.state_transfer()
                self.micro_grid.state_transfer()
        
        # Print communication efficiency statistics
        print(f'Total communication: {sum_commu}, Total number of EVs: {sum_number}')
        
        # Print final cost summary
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")
      
    def solution_local(self):
        """
        Execute local optimization strategy.
        """
        # Set up logging for local optimization
        self.micro_grid.file_path(directory='results/microgrid_local/')
        self.chargers.file_path(directory='results/record_local/')
        
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)
                
                if number > 0:
                    # Each EV optimizes independently (no coordination)
                    self.chargers.local_optimization()
                    power, _, _ = self.chargers.get_lower_variable()
                    self.micro_grid.local_result_update(power)
                    
                    # Microgrid operates without considering EV coordination
                    self.micro_grid.upper_level_update_noev()
                else:
                    self.micro_grid.upper_level_update_noev()
              
                # Execute charging and microgrid operation (no coordination)
                self.chargers.state_transfer()
                self.micro_grid.state_transfer_noev()

        # Print cost summary for local optimization
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")

    def solution_plug(self):
        """
        Execute plug-and-play charging strategy.
        """
        self.micro_grid.file_path(directory='results/microgrid_plug/')
        self.chargers.file_path(directory='results/record_plug/')
        
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)

                if number > 0:
                    # Microgrid optimization without EV coordination
                    self.micro_grid.upper_level_update_noev()
                    
                    # EVs charge at maximum power (plug-and-play)
                    self.chargers.plug_and_play_optimization()
                    power, _, _ = self.chargers.get_lower_variable()
                    self.micro_grid.local_result_update(power)
                else:
                    self.micro_grid.upper_level_update_noev()
                    
                self.chargers.state_transfer()
                self.micro_grid.state_transfer_noev()

        # Print cost summary for plug-and-play
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")

    def solution_global(self):
        """
        Execute global optimization strategy.
        """
        self.micro_grid.file_path(directory='results/microgrid_global/')
        self.chargers.file_path(directory='results/record_global/')
        
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)

                if number > 0:
                    # Global microgrid optimization with renewable maximization
                    self.micro_grid.global_optimization()
                    
                    # Communicate global constraints to EVs
                    upper_variables, dual = self.micro_grid.upper_variable_communicate()
                    self.chargers.get_upper_variable(upper_variables, dual)
                    
                    # EVs optimize within global constraints
                    self.chargers.global_optimization()
                    power, _, _ = self.chargers.get_lower_variable()
                    self.micro_grid.local_result_update(power)
                else:
                    self.micro_grid.upper_level_update_noev()
                    
                self.chargers.state_transfer()
                self.micro_grid.state_transfer_noev()

        # Print cost summary for global optimization
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")

    def solution_centralized(self):
        """
        Execute centralized optimization strategy.
        """
        self.micro_grid.file_path(directory='results/microgrid_centralized/')
        self.chargers.file_path(directory='results/record_centralized/')
        
        for day in range(363):
            for step in range(96):
                print(f"Day {day}, Step {step}")
                
                self.chargers.time_step(day, step)
                self.chargers.state_update()
                number = self.chargers.lower_level_number()
                print(f"There are {number} EV on charging")
                
                self.micro_grid.time_step(day, step)
                self.micro_grid.state_update(number)

                if number > 0:
                    # Collect detailed state information from all EVs
                    duration, energy_demand, power_max = self.chargers.get_lower_state()
                    self.micro_grid.local_state_update(duration, energy_demand, power_max)
                    
                    # Solve centralized optimization with individual EV models
                    self.micro_grid.centralized_optimization()
                    
                    # Extract individual power commands for each EV
                    power = self.micro_grid.upper_result_update()
                    self.chargers.centralized_optimization(power)
                else:
                    self.micro_grid.upper_level_update_noev()

                # Execute optimized charging and microgrid operation
                self.chargers.state_transfer()
                self.micro_grid.state_transfer()

        # Print cost summary for centralized optimization
        for i in range(len(self.chargers.chargers)):
            cost, degrad, comfort = self.chargers.chargers[i].get_cost()
            print(f"Charger {i}: Sum: {cost + degrad + comfort}, Cost: {cost}, Degradation: {degrad}, Comfort: {comfort}")
        revenue, cost = self.micro_grid.get_cost()
        print(f"Microgrid: Sum: {revenue - cost}, Grid Cost: {cost}, EV Revenue: {revenue}")