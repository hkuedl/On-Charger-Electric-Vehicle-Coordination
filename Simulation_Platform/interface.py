import numpy as np

class Interface():
    """
    Charging interface between upper-level microgrid and lower-level chargers.
    """
    
    def __init__(self, chargers, period) -> None:
        """
        Initialize the charging interface with chargers and optimization period.
        
        Args:
            chargers: List of Charger objects to be coordinated
            period: Optimization horizon period (number of time steps)
        """
        self.chargers = chargers  # List of EV charger objects
        self.period = period      # Optimization period length
      
    def file_path(self, directory):
        """
        Set up file paths for all chargers to log their charging data.
        
        Args:
            directory: Directory path where charger log files will be created
        """
        for charger in self.chargers:
            charger.file_path(directory)

    def time_step(self, day, step):
        """
        Update the current time step for all chargers in the coordination system.
        
        Args:
            day: Current simulation day
            step: Current time step within the day
        """
        for charger in self.chargers:
            charger.time_step(day, step)
      
    def state_update(self):
        """
        Update the state of all chargers.
        """
        for charger in self.chargers:
            charger.state_update()

    def state_transfer(self):
        """
        Execute one time step of charging for all active chargers0
        """
        for charger in self.chargers:
            charger.state_transfer()
      
    def lower_level_number(self):
        """
        Count the number of active chargers.
        
        Returns:
            int: Number of chargers currently in use
        """
        self.number = 0
        for charger in self.chargers:
            if charger.plug != 0:  # Check if EV is plugged in
                self.number += 1
        return self.number
      
    def lower_level_optimization(self, rho):
        """
        Execute lower-level optimization using Gurobi for all active chargers.
        
        Args:
            rho: Penalty parameter for ADMM consensus constraints
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only optimize if EV is connected
                charger.lower_level_optimization(rho)

    def lower_level_optimization_tiny(self, rho):
        """
        Execute lower-level optimization using TinyMPC for all active chargers.
        
        Args:
            rho: Penalty parameter for ADMM consensus constraints
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only optimize if EV is connected
                charger.lower_level_optimization_tiny(rho)
                 
    def local_optimization(self):
        """
        Execute local optimization for all active chargers without coordination.
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only optimize if EV is connected
                charger.local_optimization()
      
    def plug_and_play_optimization(self):
        """
        Execute plug-and-play charging strategy for all active chargers.
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only optimize if EV is connected
                charger.plug_and_play_optimization()
      
    def global_optimization(self):
        """
        Execute global optimization strategy for all active chargers.
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only optimize if EV is connected
                charger.global_optimization()
      
    def centralized_optimization(self, power):
        """
        Execute centralized optimization where power commands for all active chargers.
        
        Args:
            power: Array of power commands for each active charger
        """
        n = 0  # Counter for active chargers
        for charger in self.chargers:
            if charger.plug != 0:  # Only command if EV is connected
                charger.centralized_optimization(power[n])
                n += 1

    def get_lower_variable(self):
        """
        Collect lower-level variables from all active chargers for coordination.
        
        Returns:
            all_lower_variable: Aggregated power schedule across all chargers
            sum_communicate: Total number of chargers requesting communication
            sum_number: Total number of active chargers
        """
        all_lower_variable = np.zeros(self.period)  # Aggregated power schedule
        sum_communicate = 0  # Count of chargers requesting communication
        sum_number = 0       # Count of active chargers
        
        for charger in self.chargers:
            if charger.plug != 0:  # Only collect from active chargers
                lower_variable, communicate_sign = charger.lower_variable_communicate()
                
                # Aggregate power schedules
                for t in range(self.period):
                    all_lower_variable[t] += lower_variable[t]
                
                sum_communicate += communicate_sign  # Add communication request
                sum_number += 1
                
        return all_lower_variable, sum_communicate, sum_number

    def get_upper_variable(self, upper_variable, dual):
        """
        Distribute upper-level variables and dual prices from microgrid platform.
        
        Args:
            upper_variable: Upper-level decision variables from grid microgrid platform
            dual: Updated dual prices/multipliers from coordination algorithm
        """
        for charger in self.chargers:
            if charger.plug != 0:  # Only update active chargers
                charger.upper_variable_update(upper_variable, dual)

    def get_lower_state(self):
        """
        Collect state information from all active chargers including remaining charging duration, energy demand, and power limits.
        
        Returns:
            all_duration: Array of remaining charging durations for each active charger
            all_energy_demand: Array of energy demands for each active charger  
            all_ev_power_max: Array of maximum power limits for each active charger
        """
        # Initialize arrays based on number of active chargers
        all_duration = np.zeros(self.number)      # Remaining charging time
        all_energy_demand = np.zeros(self.number) # Energy still needed
        all_ev_power_max = np.zeros(self.number)  # Maximum charging power
        
        n = 0  # Counter for active chargers
        for charger in self.chargers:
            if charger.plug != 0:  # Only collect from active chargers
                # Get state information from charger
                duration, energy, power_max = charger.lower_state_upload()
                
                # Store in microgrid platform arrays
                all_duration[n] = duration
                all_energy_demand[n] = energy  
                all_ev_power_max[n] = power_max
                n += 1

        return all_duration, all_energy_demand, all_ev_power_max