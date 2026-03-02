import numpy as np  
import pandas as pd  
import serial  
import struct  
import time  
import csv  
import os  
from datetime import datetime  
from pyomo.environ import *  
from pyomo.opt import SolverFactory  

class MicrogridPlatform:  
    def __init__(self, serial_port='COM3', baud_rate=115200, num_chargers=120):  
        """  
        Initialize microgrid platform  
          
        Args:  
            serial_port: Serial port for MCU communication  
            baud_rate: Serial communication baud rate
            num_chargers: Total number of charger stations
        """  
        # Communication setup  
        self.serial_port = serial_port  
        self.baud_rate = baud_rate  
        self.ser = None  
        self.setup_serial()  
          
        # System parameters  
        self.period = 96 
        self.day = 0  
        self.step = 0  
        self.num_chargers = num_chargers  # Total number of chargers
        self.num_active_evs = 0  # Number of EVs currently charging
          
        # Load system data  
        self.load_system_data()  
          
        # Microgrid components  
        self.es_energy = 100.0  # Initial energy storage SOC (kWh)  
        self.es_capacity = 200.0  # Energy storage capacity (kWh)  
        self.es_power_max = 100.0  # Maximum ES power (kW)  
        self.P_max = 1500.0  # Maximum grid power (kW)  
          
        # Algorithm variables (per charger)
        self.lower_variable = np.zeros(self.period)  # Average power per charger
        self.upper_variable = np.zeros(self.period)  # Target power per charger
        self.dual = np.zeros(self.period)  # Dual prices
        self.ev_power = np.zeros(self.period)  # Optimal power per charger
        self.ev_power_old = np.zeros(self.period)  
        
        # Store all chargers' data
        self.all_chargers_power = np.zeros((self.num_chargers, self.period))
        self.chargers_status = np.zeros(self.num_chargers, dtype=bool)  # Active/inactive
          
        # Operational variables  
        self.grid_power_0 = 0.0  
        self.pv_acc_0 = 0.0  
        self.es_power_0 = 0.0  
          
        # Cost tracking  
        self.revenue = []  
        self.cost = []  
          
        # Constants  
        self.eta = 0.95  # Energy storage efficiency  
        self.delta = 0.25  # Time step (15 minutes = 0.25 hours)  
        self.M = 10000  # Big M for binary constraints  
          
        # Communication statistics
        self.total_communications = 0
        self.successful_communications = 0
        
        # Setup logging  
        self.setup_logging()  

    def setup_serial(self):  
        """Setup serial communication with MCU"""  
        try:  
            self.ser = serial.Serial(  
                port=self.serial_port,  
                baudrate=self.baud_rate,  
                bytesize=serial.EIGHTBITS,  
                parity=serial.PARITY_NONE,  
                stopbits=serial.STOPBITS_ONE,  
                timeout=2.0  
            )  
            time.sleep(0.1)  # Wait for serial to initialize
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"✓ Serial connection established on {self.serial_port}")  
        except Exception as e:  
            print(f"✗ Failed to setup serial connection: {e}")  
            self.ser = None  

    def load_system_data(self):  
        """Load base load, PV generation, and price data"""  
        try:  
            # Load data files  
            self.base_data = pd.read_csv('dataset/base_load.csv')  
            self.pv_data = pd.read_csv('dataset/pv_generation.csv')   
            self.lmp_price_data = pd.read_csv('dataset/lmp_price.csv')  
            
            # Validate data length
            min_length = self.period * 7  # At least 7 days
            if len(self.base_data) < min_length:
                raise ValueError(f"Insufficient data: need at least {min_length} points")
              
            print("✓ System data loaded successfully")  
        except Exception as e:  
            print(f"⚠ Error loading data: {e}")  
            self.create_data()  

    def create_data(self):  
        """Create data for testing"""  
        time_points = 96 * 7  # 7 days of 15-min intervals  
        time_array = np.linspace(0, 7 * 2 * np.pi, time_points)
          
        # Base load with daily pattern (50-150 kW)
        self.base_data = pd.DataFrame({  
            'day': np.repeat(range(7), 96),  
            'step': np.tile(range(96), 7),  
            'power': 100 + 30 * np.sin(time_array - np.pi/2) + 20 * np.random.randn(time_points) * 0.1
        })  
          
        # PV generation with daily pattern (0-150 kW during day)
        pv_generation = np.zeros(time_points)
        for i in range(time_points):
            hour_of_day = (i % 96) * 0.25  # Hour in day
            if 6 <= hour_of_day <= 18:  # Daytime
                pv_generation[i] = 150 * np.sin((hour_of_day - 6) * np.pi / 12) ** 2
        
        self.pv_data = pd.DataFrame({  
            'day': np.repeat(range(7), 96),  
            'step': np.tile(range(96), 7),   
            'power': pv_generation
        })  
          
        # LMP price with daily pattern (0.05-0.25 $/kWh)
        self.lmp_price_data = pd.DataFrame({  
            'day': np.repeat(range(7), 96),  
            'step': np.tile(range(96), 7),  
            'price': 0.15 + 0.08 * np.sin(time_array) + 0.02 * np.random.randn(time_points)
        })  
          
        print("✓ Using data for testing")  

    def setup_logging(self):  
        """Setup CSV logging for results"""  
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  
        self.csv_file_path = f'results/microgrid_{timestamp}.csv'  
          
        # Create results directory  
        os.makedirs('results', exist_ok=True)  
          
        # Write CSV header  
        with open(self.csv_file_path, 'w', newline='') as file:  
            writer = csv.writer(file)  
            writer.writerow([  
                'day', 'step', 'grid_price', 'dual_price', 
                'ev_power_per_charger', 'total_ev_power', 'num_active_evs',
                'base_load', 'es_power', 'es_soc', 'grid_power', 
                'pv_used', 'pv_available',
                'revenue', 'cost', 'profit'
            ])  
        
        print(f"✓ Logging to {self.csv_file_path}")

    def time_step(self, day, step):  
        """Update current time step"""  
        self.day = day  
        self.step = step  

    def load_data(self):  
        """Load data for current optimization horizon"""  
        record = self.day * 96 + self.step  
        
        # Handle wrapping for last time steps
        if record + self.period > len(self.base_data):
            # Wrap around or pad with last values
            base = np.pad(
                self.base_data['power'].iloc[record:].values,
                (0, record + self.period - len(self.base_data)),
                mode='edge'
            )
            pv = np.pad(
                self.pv_data['power'].iloc[record:].values,
                (0, record + self.period - len(self.pv_data)),
                mode='edge'
            )
            lmp_price = np.pad(
                self.lmp_price_data['price'].iloc[record:].values,
                (0, record + self.period - len(self.lmp_price_data)),
                mode='edge'
            )
        else:
            base = self.base_data['power'].iloc[record:record+self.period].values  
            pv = self.pv_data['power'].iloc[record:record+self.period].values  
            lmp_price = self.lmp_price_data['price'].iloc[record:record+self.period].values  
          
        return base, pv, lmp_price  

    def state_update(self, num_active_evs):  
        """Update microgrid state and initialize variables"""  
        base, pv, lmp_price = self.load_data()  
          
        self.base_load = base  
        self.pv_output = pv  
        self.num_active_evs = num_active_evs  
        self.grid_price = lmp_price  
          
        # Initialize algorithm variables  
        self.lower_variable = np.zeros(self.period)  
        self.ev_power_old = self.ev_power.copy()
        # Keep previous ev_power as initial guess
          
        # Update dual variables with time-shifting (MPC style)
        if self.step > 0 or self.day > 0:
            # Shift dual prices forward
            self.dual[:-1] = self.dual[1:]  
            self.dual[-1] = lmp_price[-1]
        else:  
            self.dual = lmp_price.copy()  
        
        # Ensure non-negative dual prices
        self.dual = np.maximum(self.dual, 0)
          
        # Reset current outputs  
        self.grid_power_0 = 0  
        self.pv_acc_0 = 0  
        self.es_power_0 = 0  

    def lower_variable_update(self, all_chargers_power):  
        """
        Update lower-level variables from all EV chargers
        
        Args:
            all_chargers_power: 2D array (num_chargers x period) with power schedules
        """
        self.all_chargers_power = all_chargers_power.copy()
        
        if self.num_active_evs > 0:  
            # Calculate average power per charger across active EVs
            active_mask = self.chargers_status
            if np.sum(active_mask) > 0:
                self.lower_variable = np.mean(
                    all_chargers_power[active_mask, :], 
                    axis=0
                )
            else:
                self.lower_variable = np.zeros(self.period)
        else:  
            self.lower_variable = np.zeros(self.period)  

    def upper_variable_communicate(self):  
        """Prepare data for communication to EV chargers"""  
        # Compute upper variable (consensus target)
        for t in range(self.period):  
            self.upper_variable[t] = self.ev_power[t] - self.lower_variable[t]  
        
        # Ensure non-negative dual prices
        self.dual = np.maximum(self.dual, 0)
          
        return self.upper_variable, self.dual  

    def send_upper_variables(self, upper_vars, dual_prices):  
        """Send upper variables and dual prices to MCU via serial"""  
        if self.ser is None or not self.ser.is_open:  
            print("✗ Serial connection not available")  
            return False  
          
        try:  
            # Prepare data packet  
            frame_header = 0xAA  
            command = 0x02  # CMD_RECV_UPPER  
            
            # Send only first few time steps to reduce data size (e.g., 24 steps = 6 hours)
            send_period = min(24, self.period)
            data_length = send_period * 8  # 2 floats * 4 bytes each
              
            # Build packet  
            packet = bytearray()  
            packet.append(frame_header)  
            packet.append(command)  
            packet.append(data_length & 0xFF)  # Low byte
            packet.append((data_length >> 8) & 0xFF)  # High byte
              
            # Add upper variables and dual prices  
            for t in range(send_period):  
                # Ensure valid float values
                upper_val = float(np.clip(upper_vars[t], -1000, 1000))
                dual_val = float(np.clip(dual_prices[t], 0, 10))
                
                packet.extend(struct.pack('<f', upper_val))
                packet.extend(struct.pack('<f', dual_val))
              
            # Calculate and add CRC  
            crc = self.calculate_crc16(packet[1:])  # CRC over cmd + data
            packet.extend(struct.pack('<H', crc))
              
            # Send packet  
            self.ser.write(packet)  
            self.ser.flush()  
            
            self.total_communications += 1
            self.successful_communications += 1
              
            print(f"→ Sent upper variables to MCU (period={send_period})")  
            return True  
              
        except Exception as e:  
            print(f"✗ Error sending data: {e}")  
            self.total_communications += 1
            return False  

    def receive_lower_variables(self, timeout=3.0):  
        """
        Receive lower variables from all MCU chargers via serial
        
        Returns:
            power_array: 2D array (num_chargers x period)
            success: Communication success flag
            num_received: Number of chargers that responded
        """  
        if self.ser is None or not self.ser.is_open:  
            return np.zeros((self.num_chargers, self.period)), False, 0  
          
        try:  
            start_time = time.time()
            received_data = []
            
            # Wait for multiple responses (one per charger)
            while time.time() - start_time < timeout:
                if self.ser.in_waiting > 0:  
                    # Read frame header  
                    header = self.ser.read(1)  
                    if len(header) == 0 or header[0] != 0xAA:  
                        continue
                      
                    # Read command and length  
                    cmd_len = self.ser.read(3)  
                    if len(cmd_len) != 3:
                        continue
                    
                    command = cmd_len[0]
                    if command != 0x01:  # CMD_SEND_LOWER  
                        continue
                    
                    data_length = cmd_len[1] | (cmd_len[2] << 8)
                      
                    # Read charger ID, power data, and CRC
                    data = self.ser.read(data_length + 2)  # +2 for CRC  
                    if len(data) != data_length + 2:  
                        print(f"⚠ Incomplete packet: expected {data_length + 2}, got {len(data)}")
                        continue
                      
                    # Verify CRC
                    packet_for_crc = bytearray([command]) + bytearray([cmd_len[1], cmd_len[2]]) + data[:-2]
                    calculated_crc = self.calculate_crc16(packet_for_crc)
                    received_crc = struct.unpack('<H', data[-2:])[0]
                    
                    if calculated_crc != received_crc:
                        print(f"⚠ CRC mismatch: calculated {calculated_crc:04X}, received {received_crc:04X}")
                        continue
                      
                    # Extract charger ID (first byte of data)
                    charger_id = data[0]
                    if charger_id >= self.num_chargers:
                        print(f"⚠ Invalid charger ID: {charger_id}")
                        continue
                    
                    # Extract power values (floats after charger ID)
                    num_values = (data_length - 1) // 4  # -1 for charger_id byte
                    power_values = np.zeros(self.period)
                    
                    for i in range(min(num_values, self.period)):
                        offset = 1 + i * 4  # +1 to skip charger_id
                        power_values[i] = struct.unpack('<f', data[offset:offset+4])[0]
                    
                    received_data.append({
                        'charger_id': charger_id,
                        'power': power_values
                    })
                    
                    print(f"← Received data from charger {charger_id}")
                
                # Check if we received enough data
                if len(received_data) >= self.num_active_evs:
                    break
                
                time.sleep(0.01)  # Small delay before checking again
            
            # Process received data
            if len(received_data) == 0:
                print("✗ No data received from chargers")
                return np.zeros((self.num_chargers, self.period)), False, 0
            
            # Initialize power array
            power_array = np.zeros((self.num_chargers, self.period))
            self.chargers_status = np.zeros(self.num_chargers, dtype=bool)
            
            # Fill in received data
            for data_point in received_data:
                charger_id = data_point['charger_id']
                power_array[charger_id, :] = data_point['power']
                self.chargers_status[charger_id] = True
            
            num_received = len(received_data)
            print(f"✓ Received data from {num_received}/{self.num_active_evs} active chargers")
            
            return power_array, True, num_received
              
        except Exception as e:  
            print(f"✗ Error receiving data: {e}")  
            import traceback
            traceback.print_exc()
          
        return np.zeros((self.num_chargers, self.period)), False, 0

    def calculate_crc16(self, data):  
        """Calculate CRC16-CCITT for data integrity"""  
        crc = 0xFFFF  
        for byte in data:  
            crc ^= byte  
            for _ in range(8):  
                if crc & 0x0001:  
                    crc = (crc >> 1) ^ 0xA001  
                else:  
                    crc >>= 1  
        return crc & 0xFFFF

    def upper_level_update_noev(self):  
        """Solve microgrid optimization without EV coordination"""  
        model = ConcreteModel()  
        model.T = RangeSet(0, self.period - 1)
          
        # Parameters  
        model.price_g = Param(model.T, initialize={t: float(self.grid_price[t]) for t in range(self.period)})  
        model.P_base = Param(model.T, initialize={t: float(self.base_load[t]) for t in range(self.period)})  
        model.P_pv = Param(model.T, initialize={t: float(self.pv_output[t]) for t in range(self.period)})  
        model.P_max = Param(initialize=float(self.P_max))  
        model.ES_cap = Param(initialize=float(self.es_capacity))
        model.ES_max = Param(initialize=float(self.es_power_max))
        model.ES_init = Param(initialize=float(self.es_energy))
          
        # Variables  
        model.P_g = Var(model.T, within=NonNegativeReals, bounds=(0, self.P_max))  
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)  
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_power_max))  
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_power_max))  
        model.U_es = Var(model.T, within=Binary)  
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_capacity))  
          
        # Objective: minimize grid cost
        def obj_rule(model):  
            return sum(model.price_g[t] * model.P_g[t] * self.delta for t in model.T)  
        model.objective = Objective(rule=obj_rule, sense=minimize)  
          
        # Constraints  
        def power_balance_rule(model, t):  
            return (model.P_g[t] + model.P_pv_acc[t] + model.P_es_dis[t] ==   
                   model.P_base[t] + model.P_es_ch[t])  
        model.power_balance = Constraint(model.T, rule=power_balance_rule)  
          
        def pv_limit_rule(model, t):  
            return model.P_pv_acc[t] <= model.P_pv[t]  
        model.pv_limit = Constraint(model.T, rule=pv_limit_rule)  
          
        def storage_dynamics_rule(model, t):  
            if t == 0:  
                es_prev = model.ES_init
            else:  
                es_prev = model.E_es[t-1]  
            return (model.E_es[t] == es_prev +   
                   self.eta * self.delta * model.P_es_ch[t] -  
                   (self.delta / self.eta) * model.P_es_dis[t])  
        model.storage_dynamics = Constraint(model.T, rule=storage_dynamics_rule)  
          
        def charge_limit_rule(model, t):  
            return model.P_es_ch[t] <= model.ES_max * model.U_es[t]  
        model.charge_limit = Constraint(model.T, rule=charge_limit_rule)  
          
        def discharge_limit_rule(model, t):  
            return model.P_es_dis[t] <= model.ES_max * (1 - model.U_es[t])  
        model.discharge_limit = Constraint(model.T, rule=discharge_limit_rule)  
          
        # Solve  
        try:  
            opt = SolverFactory('gurobi')
            opt.options['MIPGap'] = 0.01  # 1% optimality gap
            opt.options['TimeLimit'] = 30  # 30 seconds time limit
            solution = opt.solve(model, tee=False)
              
            if solution.solver.termination_condition == TerminationCondition.optimal or \
               solution.solver.termination_condition == TerminationCondition.feasible:
                self.grid_power_0 = value(model.P_g[0])  
                self.pv_acc_0 = value(model.P_pv_acc[0])  
                self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])  
                
                print(f"✓ Optimization (no EV): Grid={self.grid_power_0:.2f} kW, ES={self.es_power_0:.2f} kW, PV={self.pv_acc_0:.2f} kW")
                return True  
            else:  
                print(f"✗ Optimization failed: {solution.solver.termination_condition}")  
                # Use fallback values
                self.grid_power_0 = self.base_load[0]
                self.pv_acc_0 = min(self.pv_output[0], self.base_load[0])
                self.es_power_0 = 0
                return False  
                  
        except Exception as e:  
            print(f"✗ Optimization error: {e}")  
            import traceback
            traceback.print_exc()
            # Use fallback values
            self.grid_power_0 = self.base_load[0]
            self.pv_acc_0 = min(self.pv_output[0], self.base_load[0])
            self.es_power_0 = 0
            return False  

    def upper_level_update(self, rho):  
        """Solve upper-level optimization with EV coordination"""  
        model = ConcreteModel()  
        model.T = RangeSet(0, self.period - 1)
          
        # Parameters  
        model.price_g = Param(model.T, initialize={t: float(self.grid_price[t]) for t in range(self.period)})  
        model.P_base = Param(model.T, initialize={t: float(self.base_load[t]) for t in range(self.period)})  
        model.P_pv = Param(model.T, initialize={t: float(self.pv_output[t]) for t in range(self.period)})  
        model.P_max = Param(initialize=float(self.P_max))  
        model.number = Param(initialize=max(1, int(self.num_active_evs)))  
        model.lower_var = Param(model.T, initialize={t: float(self.lower_variable[t]) for t in range(self.period)})  
        model.dual_price = Param(model.T, initialize={t: float(self.dual[t]) for t in range(self.period)})  
        model.ES_cap = Param(initialize=float(self.es_capacity))
        model.ES_max = Param(initialize=float(self.es_power_max))
        model.ES_init = Param(initialize=float(self.es_energy))
          
        # Variables  
        model.P_g = Var(model.T, within=NonNegativeReals, bounds=(0, self.P_max))  
        model.P_e = Var(model.T, within=NonNegativeReals, bounds=(0, 50))  # EV power per charger (max 50 kW)
        model.P_pv_acc = Var(model.T, within=NonNegativeReals)  
        model.P_es_ch = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_power_max))  
        model.P_es_dis = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_power_max))  
        model.U_es = Var(model.T, within=Binary)  
        model.E_es = Var(model.T, within=NonNegativeReals, bounds=(0, self.es_capacity))  
          
        # Objective with consensus penalty  
        def obj_rule(model):  
            # Grid cost
            cost_grid = sum(model.price_g[t] * model.P_g[t] * self.delta for t in model.T)  
            # EV revenue (negative cost)
            revenue_ev = sum(-model.dual_price[t] * (model.P_e[t] * model.number) * self.delta for t in model.T)  
            # ADMM consensus penalty
            penalty = sum(model.number * (rho/2) * (model.P_e[t] - model.lower_var[t])**2 for t in model.T)  
            return cost_grid + revenue_ev + penalty  
        model.objective = Objective(rule=obj_rule, sense=minimize)  
          
        # Constraints
        def power_balance_rule(model, t):  
            return (model.P_g[t] + model.P_pv_acc[t] + model.P_es_dis[t] ==   
                   model.P_base[t] + model.P_es_ch[t] + model.P_e[t] * model.number)  
        model.power_balance = Constraint(model.T, rule=power_balance_rule)  
          
        def pv_limit_rule(model, t):  
            return model.P_pv_acc[t] <= model.P_pv[t]  
        model.pv_limit = Constraint(model.T, rule=pv_limit_rule)  
          
        def storage_dynamics_rule(model, t):  
            if t == 0:  
                es_prev = model.ES_init
            else:  
                es_prev = model.E_es[t-1]  
            return (model.E_es[t] == es_prev +   
                   self.eta * self.delta * model.P_es_ch[t] -  
                   (self.delta / self.eta) * model.P_es_dis[t])  
        model.storage_dynamics = Constraint(model.T, rule=storage_dynamics_rule)  
          
        def charge_limit_rule(model, t):  
            return model.P_es_ch[t] <= model.ES_max * model.U_es[t]  
        model.charge_limit = Constraint(model.T, rule=charge_limit_rule)  
          
        def discharge_limit_rule(model, t):  
            return model.P_es_dis[t] <= model.ES_max * (1 - model.U_es[t])  
        model.discharge_limit = Constraint(model.T, rule=discharge_limit_rule)  
          
        # Solve  
        try:  
            opt = SolverFactory('gurobi')
            opt.options['MIPGap'] = 0.01
            opt.options['TimeLimit'] = 30
            solution = opt.solve(model, tee=False)
              
            if solution.solver.termination_condition == TerminationCondition.optimal or \
               solution.solver.termination_condition == TerminationCondition.feasible:
                # Update EV power schedule  
                self.ev_power_old = self.ev_power.copy()  
                for t in range(self.period):  
                    self.ev_power[t] = value(model.P_e[t])  
                  
                # Update current outputs  
                self.grid_power_0 = value(model.P_g[0])  
                self.pv_acc_0 = value(model.P_pv_acc[0])  
                self.es_power_0 = value(model.P_es_ch[0]) - value(model.P_es_dis[0])  
                
                print(f"✓ Optimization (EV): Grid={self.grid_power_0:.2f} kW, ES={self.es_power_0:.2f} kW, " +
                      f"PV={self.pv_acc_0:.2f} kW, EV={self.ev_power[0]:.2f} kW/charger")
                return True  
            else:  
                print(f"✗ Optimization failed: {solution.solver.termination_condition}")  
                return False  
                  
        except Exception as e:  
            print(f"✗ Optimization error: {e}")  
            import traceback
            traceback.print_exc()
            return False  

    def dual_update(self, rho):  
        """Update dual variables (prices) using gradient ascent"""  
        for t in range(self.period):  
            # Gradient step
            self.dual[t] += rho * (self.ev_power[t] - self.lower_variable[t])  
            # Project to non-negative prices
            self.dual[t] = max(0.0, self.dual[t])

    def compute_residual(self, rho):  
        """Compute primal and dual residuals for convergence check"""  
        # Primal residual: consensus constraint violation (per charger)
        primal_residual = np.linalg.norm(self.ev_power - self.lower_variable)  
          
        # Dual residual: change in upper variables (scaled by rho)
        if self.num_active_evs > 0:
            dual_residual = rho * self.num_active_evs * np.linalg.norm(self.ev_power - self.ev_power_old)
        else:
            dual_residual = 0.0
          
        return primal_residual, dual_residual  

    def state_transfer(self):  
        """Execute one time step and update system state"""  
        # Update energy storage state
        delta_energy = self.es_power_0 * self.delta
        
        if self.es_power_0 >= 0:  # Charging
            self.es_energy += self.eta * delta_energy
        else:  # Discharging
            self.es_energy += delta_energy / self.eta
        
        # Clip to valid range
        self.es_energy = np.clip(self.es_energy, 0, self.es_capacity)
          
        # Calculate financial flows
        if self.num_active_evs > 0:  
            # Revenue from selling power to EVs
            revenue = self.dual[0] * self.ev_power[0] * self.num_active_evs * self.delta  
        else:  
            revenue = 0.0
            
        # Cost of buying power from grid
        cost = self.grid_price[0] * self.grid_power_0 * self.delta  
          
        self.revenue.append(revenue)  
        self.cost.append(cost)  
        
        profit = revenue - cost
          
        # Log to CSV  
        try:
            with open(self.csv_file_path, 'a', newline='') as file:  
                writer = csv.writer(file)  
                writer.writerow([  
                    self.day, 
                    self.step,  
                    f"{self.grid_price[0]:.4f}",
                    f"{self.dual[0]:.4f}",  
                    f"{self.ev_power[0]:.4f}",
                    f"{self.ev_power[0] * self.num_active_evs:.4f}",
                    self.num_active_evs,
                    f"{self.base_load[0]:.2f}",  
                    f"{self.es_power_0:.2f}",
                    f"{self.es_energy:.2f}",  
                    f"{self.grid_power_0:.2f}",  
                    f"{self.pv_acc_0:.2f}",  
                    f"{self.pv_output[0]:.2f}",  
                    f"{revenue:.4f}",
                    f"{cost:.4f}",
                    f"{profit:.4f}"
                ])
        except Exception as e:
            print(f"⚠ Logging error: {e}")
            
        # Print summary
        print(f"  State: ES_SOC={self.es_energy:.1f}/{self.es_capacity:.1f} kWh, " +
              f"Revenue=${revenue:.2f}, Cost=${cost:.2f}, Profit=${profit:.2f}")

    def get_cost(self):  
        """Get total revenue and cost"""  
        total_revenue = sum(self.revenue)  
        total_cost = sum(self.cost)  
        return total_revenue, total_cost  

    def close(self):  
        """Clean up resources"""  
        if self.ser and self.ser.is_open:  
            self.ser.close()  
        
        # Print communication statistics
        if self.total_communications > 0:
            success_rate = 100 * self.successful_communications / self.total_communications
            print(f"\n{'='*60}")
            print(f"Communication Statistics:")
            print(f"  Total attempts: {self.total_communications}")
            print(f"  Successful: {self.successful_communications}")
            print(f"  Success rate: {success_rate:.1f}%")
            print(f"{'='*60}")
        
        print("✓ Microgrid platform closed")  

def main():  
    """Main execution function"""  
    print("="*60)
    print("Microgrid EV Charging Coordination Platform")
    print("="*60)
    
    # Create microgrid platform  
    platform = MicrogridPlatform(
        serial_port='COM3', 
        baud_rate=115200,
        num_chargers=120  # Total number of chargers
    )
      
    # ADMM algorithm parameters  
    rho_init = 0.5  # Initial penalty parameter
    max_iter = 30  # Maximum ADMM iterations per time step
    primal_tol = 0.5  # Primal residual tolerance
    dual_tol = 0.5  # Dual residual tolerance
    mu = 10.0  # Residual balancing parameter
    tau = 2.0  # Penalty update factor
    rho_min = 1e-2  
    rho_max = 1e2  
    check_interval = 5  # Check for penalty update every N iterations
      
    # Communication tracking  
    total_commu_success = 0  
    total_evs_served = 0  
    
    # Simulation parameters
    num_days = 2 
    steps_per_day = 96  # 15-minute intervals
      
    try:  
        print(f"\nStarting simulation: {num_days} days, {steps_per_day} steps/day")
        print("="*60 + "\n")
        
        # Main simulation loop  
        for day in range(num_days):  
            print(f"\n{'='*60}")
            print(f"DAY {day}")
            print(f"{'='*60}")
            
            for step in range(steps_per_day):  
                print(f"\n[Day {day}, Step {step}/{steps_per_day-1}] Time: {step*0.25:.2f}h")
                print("-"*60)
                  
                # Update time step  
                platform.time_step(day, step)  
                  
                # Simulate number of active EVs (in real system, query from MCU)
                # Realistic pattern: more EVs during evening hours (17:00-22:00)
                hour = step * 0.25
                if 17 <= hour <= 22:
                    num_active_evs = np.random.randint(3, 8)  # 3-7 EVs in evening
                elif 7 <= hour <= 9 or 12 <= hour <= 14:
                    num_active_evs = np.random.randint(1, 4)  # 1-3 EVs during commute/lunch
                else:
                    num_active_evs = np.random.randint(0, 2)  # 0-1 EVs at night/early morning
                  
                # Update system state
                platform.state_update(num_active_evs)  
                print(f"Active EVs: {num_active_evs}/{platform.num_chargers}")  
                  
                if num_active_evs == 0:  
                    # No EVs, solve simple optimization
                    print("No EVs charging - solving basic optimization")
                    success = platform.upper_level_update_noev()  
                    
                    if not success:
                        print("⚠ Using fallback values")
                        
                else:  
                    # Distributed optimization with ADMM
                    print(f"Starting ADMM optimization (max_iter={max_iter})")
                    rho = rho_init  
                    
                    # Initialize upper variables
                    upper_vars, dual_prices = platform.upper_variable_communicate()  
                      
                    # Send initial upper variables to MCU  
                    send_success = platform.send_upper_variables(upper_vars, dual_prices)  
                    
                    if not send_success:
                        print("⚠ Failed to send to MCU, using no-EV optimization")
                        platform.upper_level_update_noev()
                        platform.state_transfer()
                        continue
                      
                    converged = False
                    
                    for iteration in range(max_iter):  
                        print(f"\n  Iteration {iteration+1}/{max_iter} (ρ={rho:.3f})")
                        
                        # Wait for MCU to optimize (adjust based on MCU processing time)
                        time.sleep(0.2)  
                          
                        # Receive lower variables from all chargers
                        power_array, comm_success, num_received = platform.receive_lower_variables(timeout=2.0)  
                        
                        if comm_success:
                            total_commu_success += 1  
                            total_evs_served += num_received  
                            print(f"  ✓ Received from {num_received} chargers")
                        else:
                            print(f"  ✗ Communication failed")
                            # Use previous values or break
                            if iteration == 0:
                                print("  First iteration failed, using no-EV optimization")
                                platform.upper_level_update_noev()
                                break
                            else:
                                print("  Using previous iteration values")
                          
                        # Update microgrid with average power from all chargers
                        platform.lower_variable_update(power_array)  
                        
                        # Solve upper-level problem
                        opt_success = platform.upper_level_update(rho)  
                        
                        if not opt_success:
                            print("  ✗ Upper-level optimization failed")
                            if iteration > 0:
                                print("  Using previous solution")
                                break
                            else:
                                platform.upper_level_update_noev()
                                break
                        
                        # Update dual variables
                        platform.dual_update(rho)  
                          
                        # Prepare and send updated variables to MCU  
                        upper_vars, dual_prices = platform.upper_variable_communicate()  
                        platform.send_upper_variables(upper_vars, dual_prices)  
                          
                        # Check convergence  
                        primal_res, dual_res = platform.compute_residual(rho)  
                        print(f"  Residuals: primal={primal_res:.3e}, dual={dual_res:.3e}")  
                          
                        if primal_res < primal_tol and dual_res < dual_tol:  
                            print(f"  ✓ Converged at iteration {iteration+1}")
                            converged = True
                            break  
                          
                        # Adaptive penalty adjustment  
                        if iteration > 0 and (iteration + 1) % check_interval == 0:  
                            if primal_res > mu * dual_res:  
                                # Primal residual too large, increase penalty
                                rho_new = min(tau * rho, rho_max)  
                                if rho_new != rho:
                                    print(f"  ↑ ρ: {rho:.3f} → {rho_new:.3f} (primal large)")
                                    rho = rho_new
                            elif dual_res > mu * primal_res:  
                                # Dual residual too large, decrease penalty
                                rho_new = max(rho / tau, rho_min)  
                                if rho_new != rho:
                                    print(f"  ↓ ρ: {rho:.3f} → {rho_new:.3f} (dual large)")
                                    rho = rho_new
                    
                    if not converged:
                        print(f"  ⚠ Max iterations reached without convergence")
                  
                # Execute time step and update state
                platform.state_transfer()  
                  
                # Small delay between time steps  
                time.sleep(0.05)  
          
        # Print final results
        print("\n" + "="*60)
        print("SIMULATION COMPLETE")
        print("="*60)
        
        print(f"\nCommunication Statistics:")
        print(f"  Successful communications: {total_commu_success}")
        print(f"  Total EVs served: {total_evs_served}")
        
        revenue, cost = platform.get_cost()  
        profit = revenue - cost
        
        print(f"\nFinancial Summary:")
        print(f"  Total Grid Cost:  ${cost:,.2f}")
        print(f"  Total EV Revenue: ${revenue:,.2f}")
        print(f"  Net Profit:       ${profit:,.2f}")
        print(f"  ROI:              {100*profit/max(cost, 1):.2f}%")
        
        print(f"\nResults saved to: {platform.csv_file_path}")
        print("="*60)
          
    except KeyboardInterrupt:  
        print("\n\n⚠ Simulation interrupted by user")  
    except Exception as e:  
        print(f"\n\n✗ Error during simulation: {e}")  
        import traceback
        traceback.print_exc()
    finally:  
        platform.close()  

if __name__ == "__main__":  
    main()