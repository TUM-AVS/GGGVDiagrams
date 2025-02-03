import numpy as np
import sys
import os
import pandas as pd
import toml
import shutil
import matplotlib.pyplot as plt
from vehicle_models_gggv_sim_py import VehicleModelValidation
from tum_sim_types_py import ExternalInfluences, DriverInput
 

"""
Created by: Frederik Werner
Created on: 27.02.2024
"""

def param_load(gggv_config_file, sim_config_file, veh_config_file, folder, path_params):
    """
    Loads configuration parameters from specified TOML files and returns them as a dictionary.

    This function reads the provided GGGV, simulation, and vehicle configuration files, 
    appends the '.toml' extension to each filename, and loads their contents into a 
    dictionary. Additionally, it calculates some simulation parameters based on the 
    simulation configuration and stores them in the parameters dictionary. The configuration 
    files are then copied to a specified folder.

    Args:
        gggv_config_file (str): The filename of the GGGV configuration file (without extension).
        sim_config_file (str): The filename of the simulation configuration file (without extension).
        veh_config_file (str): The filename of the vehicle configuration file (without extension).
        folder (str): The destination folder where the configuration files will be copied.
        path_params (str): The directory path where the configuration files are located.

    Returns:
        dict: A dictionary containing the loaded configuration parameters, including calculated 
              simulation parameters, with keys 'gggv_config', 'sim_config', 'veh_config', and 'folder'.

    Raises:
        FileNotFoundError: If any of the specified configuration files do not exist.
    """
        
    # load params
    gggv_config_file = gggv_config_file + ".toml"
    sim_config_file = sim_config_file + ".toml"
    veh_config_file = veh_config_file + ".toml"

    # assemble path to param files
    path_params_copy = os.path.join(folder, 'params')
    params = {}
    params['folder'] = folder

    # read gggv config file
    try:
        with open(os.path.join(path_params, gggv_config_file), 'r') as fh:
            params['gggv_config'] = toml.load(fh)
            shutil.copy(os.path.join(path_params, gggv_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('album config file does not exist or invalid target specifier "'
                                + gggv_config_file + '" provided!') from None


    # read sim config file
    try:
        with open(os.path.join(path_params, sim_config_file), 'r') as fh:
            params['sim_config'] = toml.load(fh)
            shutil.copy(os.path.join(path_params, sim_config_file), path_params_copy)
            # calc additional params
            params['sim_config']['n_sim'] = round(params['sim_config']['sim_stop_time'] / params['sim_config']['t_s'])
            params['sim_config']['n_wait_PID'] = round(params['sim_config']['wait_PID_s'] / params['sim_config']['t_s'])
            params['sim_config']['n_mean'] = round(params['sim_config']['mean_s'] / params['sim_config']['t_s'])
            params['sim_config']['n_wait_f_ext'] = round(params['sim_config']['wait_f_ext_s']  / params['sim_config']['t_s'])
            params['sim_config']['velPIDparams'][0] = params['sim_config']['t_s']


    except FileNotFoundError:
        raise FileNotFoundError('sim config file does not exist or invalid target specifier "'
                                + sim_config_file + '" provided!') from None


    # read vehicle config file
    try:
        with open(os.path.join(path_params, veh_config_file), 'r') as fh:
            params['veh_config'] = toml.load(fh)
            shutil.copy(os.path.join(path_params, veh_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('vehicle config file does not exist or invalid target specifier "'
                                + veh_config_file + '" provided!') from None
    return params


class sim_handler:

    def __init__(self, params):

        """
        Constructor for the sim_handler class.

        This function takes in a dictionary containing the loaded configuration parameters 
        and stores them in the object's attributes. It also initializes the vehicle object 
        and calls the get_veh_params function to set its parameters.

        Args:
            params (dict): A dictionary containing the loaded configuration parameters.
        """
        self.gggv_config = params['gggv_config'] 
        self.sim_config = params['sim_config'] 
        self.veh_config = params['veh_config']
        self.folder = params['folder']

        # Inititalize vehicle    
        self.veh = VehicleModelValidation()
        self.get_veh_params()  

    def set_parameter(self, v_set, ax_set, gt_set):
        
        """
        This function sets the simulation parameters for the current simulation run.

        Args:
            v_set (float): The target velocity.
            ax_set (float): The target acceleration.
            gt_set (float): The target vertical acceleration.

        Returns:
            None
        """

        # Disable Prints
        sys.stdout = open(os.devnull, 'w')

        self.v_set=v_set
        self.ax_set=ax_set
        self.gt_set=gt_set

        # set integration step size
        self.pm.set_parameter_value("integration_step_size_s", np.float64(self.sim_config['t_s']))

        # set initial velocity and wheel rotation
        self.pm.set_parameter_value("initial_state.vehicle_dynamics.v_x_mps", np.float64(self.v_set))
        u_front = self.r_front*2*np.pi
        u_rear = self.r_rear*2*np.pi
        omega_front = (self.v_set/u_front)*2*np.pi*1.0
        omega_rear = (self.v_set/u_rear)*2*np.pi*1.0
        self.pm.set_parameter_value("initial_state.drivetrain.omega_FL_radps", np.float64(omega_front))
        self.pm.set_parameter_value("initial_state.drivetrain.omega_FR_radps", np.float64(omega_front))
        self.pm.set_parameter_value("initial_state.drivetrain.omega_RL_radps", np.float64(omega_rear))
        self.pm.set_parameter_value("initial_state.drivetrain.omega_RR_radps", np.float64(omega_rear))

        #set additional vehicle parameter
        if len(self.veh_config["VEH"]) != 0:
            #enable prints
            sys.stdout = sys.__stdout__
            for key, value in self.veh_config["VEH"].items():
                if key.startswith("variable"):
                    name = value
                elif key.startswith("value"):
                    param = value
                    self.pm.set_parameter_value(name, param)
                else:
                    sys.exit('Check veh_config_file section [VEH]. There might be issue with parameter order or naming.')
        else:
            print('baseline vehicle parameter loaded')
            #enable prints
            sys.stdout = sys.__stdout__

        self.veh.reset()

    def get_veh_params(self):
        """
        Loads and assigns the vehicle parameters from the parameter manager.

        This function retrieves various parameters related to the vehicle's 
        dynamics, such as mass, front and rear axle distances, track width, 
        and wheel radii, and stores them as attributes of the instance.

        Attributes set:
            mass_veh (float): Vehicle mass.
            l_f (float): Distance from the front axle to the center of gravity.
            l_r (float): Distance from the rear axle to the center of gravity.
            tw_r (float): Rear track width.
            r_front (float): Front wheel rolling radius.
            r_rear (float): Rear wheel rolling radius.
        """

        # Disable Prints
        sys.stdout = open(os.devnull, 'w')

        #load param handler
        self.pm = self.veh.get_param_manager()

        #load vehicle parameter
        self.mass_veh = self.pm.get_parameter_value("vehicle_dynamics_double_track.m").as_double()
        self.l_f = self.pm.get_parameter_value("vehicle_dynamics_double_track.l_f").as_double()
        self.l_r = self.pm.get_parameter_value("vehicle_dynamics_double_track.l").as_double() - self.l_f
        self.tw_r = self.pm.get_parameter_value("vehicle_dynamics_double_track.b_r").as_double()
        self.r_front = self.pm.get_parameter_value("vehicle_dynamics_double_track.rr_w_f").as_double()
        self.r_rear = self.pm.get_parameter_value("vehicle_dynamics_double_track.rr_w_r").as_double()
        #enable prints
        sys.stdout = sys.__stdout__
    
    def init_result_arrays(self):
        """
        Initializes result arrays and other simulation-related attributes.

        This function sets up various arrays and attributes used to store and
        track simulation results, such as velocities, accelerations, angular
        displacements, and error metrics. It also initializes parameters
        related to external influences, driver input, and simulation state.

        Attributes initialized:
            delta (np.ndarray): Array to store delta values for each simulation step.
            vx_mps (np.ndarray): Array to store longitudinal velocities in meters per second.
            vy_mps (np.ndarray): Array to store lateral velocities in meters per second.
            v_mps (np.ndarray): Array to store resultant velocities in meters per second.
            v_err (np.ndarray): Array to store velocity errors.
            ay_mps2 (np.ndarray): Array to store lateral accelerations in meters per second squared.
            ay_vf_mps2 (np.ndarray): Array to store lateral accelerations in velocity frame.
            ay_mps2_max (float): Maximum lateral acceleration encountered.
            ax_mps2 (np.ndarray): Array to store longitudinal accelerations in meters per second squared.
            ax_mean_mps2 (np.ndarray): Array to store mean longitudinal accelerations.
            dpsi_radps (np.ndarray): Array to store yaw rate in radians per second.
            beta_rad (np.ndarray): Array to store sideslip angle in radians.
            dpsi_radps_ss (np.ndarray): Array to store steady-state yaw rate.
            dpsi_mean_radps_ss (np.ndarray): Array to store mean steady-state yaw rate.
            dpsi_mean_radps (np.ndarray): Array to store mean yaw rate.
            dpsi_radps_err (np.ndarray): Array to store yaw rate errors.
            ddpsi_radps_err (np.ndarray): Array to store yaw rate change errors.
            drive_torque (np.ndarray): Array to store drive torque values.
            time_vec (list): Time vector for each simulation step.
            n_wait_PID (int): Number of steps to wait for PID stabilization.
            n_mean (int): Number of steps over which to compute mean values.
            n_wait_f_ext (int): Number of steps to wait for external force stabilization.
            drive (bool): Flag indicating if driving force is applied.
            f_ext_ok (bool): Flag indicating if external forces are stabilized.
            f_ext_x (np.ndarray): Array to store external force in x direction.
            f_ext (ExternalInfluences): Object for handling external influences.
            driver_input (DriverInput): Object for handling driver inputs.
            start_delta_ramp (bool): Flag indicating if delta ramp has started.
            start_delta_ramp_i (int): Index at which delta ramp starts.
            step_response_ok (bool): Flag indicating if step response is stabilized.
            step_response_complete (bool): Flag indicating if step response is complete.
            start_step_response_test_i (int): Index at which step response test starts.
            ay_gradient (float): Gradient of lateral acceleration.
            ol_unstable (bool): Flag indicating open-loop instability.
            ax_unfeasible (bool): Flag indicating unfeasible longitudinal acceleration.
        """

        self.delta=np.zeros(self.sim_config['n_sim'])
        self.vx_mps=np.zeros(self.sim_config['n_sim'])
        self.vy_mps=np.zeros(self.sim_config['n_sim'])
        self.v_mps=np.zeros(self.sim_config['n_sim'])
        self.v_err=np.zeros(self.sim_config['n_sim'])
        self.ay_mps2=np.zeros(self.sim_config['n_sim'])
        self.ay_vf_mps2=np.zeros(self.sim_config['n_sim'])
        self.ay_mps2_max=0.0
        self.ax_mps2=np.zeros(self.sim_config['n_sim'])
        self.ax_mean_mps2=np.zeros(self.sim_config['n_sim'])
        self.dpsi_radps=np.zeros(self.sim_config['n_sim'])
        self.beta_rad=np.zeros(self.sim_config['n_sim'])
        self.dpsi_radps_ss=np.zeros(self.sim_config['n_sim'])
        self.dpsi_mean_radps_ss=np.zeros(self.sim_config['n_sim'])
        self.dpsi_mean_radps=np.zeros(self.sim_config['n_sim'])
        self.dpsi_radps_err=np.zeros(self.sim_config['n_sim'])
        self.ddpsi_radps_err=np.zeros(self.sim_config['n_sim'])
        self.drive_torque=np.zeros(self.sim_config['n_sim'])
        self.time_vec=[self.sim_config['t_s']* t for t in range(0, self.sim_config['n_sim'])]
        self.n_wait_PID=self.sim_config['n_wait_PID']
        self.n_mean=self.sim_config['n_mean']
        self.n_wait_f_ext=self.sim_config['n_wait_f_ext']
        self.drive=True
        self.f_ext_ok=False
        self.f_ext_x = np.zeros(self.sim_config['n_sim'])
        self.f_ext = ExternalInfluences()
        self.driver_input = DriverInput()
        self.start_delta_ramp=  False
        self.start_delta_ramp_i = 0
        self.step_response_ok = False
        self.step_response_complete = False
        self.start_step_response_test_i = 0
        self.ay_gradient = 0.0
        self.ol_unstable = False
        self.ax_unfeasible = False

    def set_ext_f(self, i):
        """
        This function sets the external force for the simulation. The force is calculated based on the acceleration and vertical force set points.
        The force is ramped up over 2000 time steps to improve simuation stability
        """
        fx_ext = -self.mass_veh * self.ax_set
        fz_ext = -(self.gt_set-9.81)*self.mass_veh
        if i>self.n_wait_f_ext and i-self.n_wait_f_ext<2000:
            self.f_ext.external_force_N.x = fx_ext*(i-self.n_wait_f_ext)/2000
            self.f_ext.external_force_N.z = fz_ext*(i-self.n_wait_f_ext)/2000
        elif i+self.n_wait_f_ext >= 200:
            self.f_ext.external_force_N.x = fx_ext
            self.f_ext.external_force_N.z  = fz_ext
            self.f_ext_ok=True
        self.f_ext_x[i] = self.f_ext.external_force_N.x
        self.veh.set_external_influences(self.f_ext)

    def set_driver_input(self, i):
        """
        Sets the driver input for the simulation at the given index.

        This function adjusts the steering angle and drive torque based on the current
        simulation state, including whether a delta ramp or step response test is in progress.
        It calculates and applies the appropriate steering angle and wheel torques.

        Parameters:
            i (int): The current simulation step index.

        Behavior:
            - If a delta ramp is active, increment the steering angle by a predefined rise rate.
            - During a step response test, calculate the steering angle incrementally until a target
            angle is reached, analyze the system's response, and adjust the delta rise rate based
            on the lateral acceleration gradient.
            - Set the appropriate torque to the wheels based on the drive or brake torque required
            for the current step.
            - Updates the driver input for the vehicle with the calculated steering angle and torque values.
        """

        if self.start_delta_ramp:
            self.delta[i]=self.delta[i-1]+(self.sim_config["delta_rise_rate"]*self.sim_config['t_s'])
        
        # step response test
        if self.f_ext_ok and not self.start_delta_ramp and np.abs(self.v_err[i])<0.05 and np.abs(self.ax_mean_mps2[i])<0.1 :
            # set step response delta
            if self.delta[i-1] < self.sim_config['delta_step_test']:
                self.delta[i] = self.delta[i-1] + self.sim_config['delta_step_test'] / (self.sim_config['wait_step_time'] / 2 / self.sim_config['t_s'])
            else:
                self.delta[i] = self.sim_config['delta_step_test'] 
            
            # save step response start
            if self.start_step_response_test_i == 0:
                self.start_step_response_test_i = i
            # wait and analyze step response
            elif i == self.start_step_response_test_i + self.sim_config['wait_step_time'] / self.sim_config['t_s']:
                if self.ay_vf_mps2[i] == 0:
                    test=1
                self.ay_gradient = self.ay_vf_mps2[i] / self.sim_config['delta_step_test']
                self.sim_config['delta_rise_rate'] = self.sim_config['delta_step_test'] / self.ay_vf_mps2[i] * self.sim_config['ay_target_rate']
            # reset delta to 0
            elif i > self.start_step_response_test_i + self.sim_config['wait_step_time'] / self.sim_config['t_s']:
                self.delta[i] = 0.0
                self.step_response_complete = True
            
            # wait again until vehicle is settled
            if self.ay_vf_mps2[i]<0.01 and i>self.start_step_response_test_i+2*self.sim_config['wait_step_time'] / self.sim_config['t_s']:
                self.step_response_ok = True

        
        self.driver_input.steering_angle_rad = self.delta[i]

        # calc drive torque
        if self.drive_torque[i] >= 0:
            self.driver_input.wheel_torque_Nm.front_left  = (0.5 * self.drive_torque[i]) * (1-self.veh_config["cDiff"])
            self.driver_input.wheel_torque_Nm.front_right = (0.5 * self.drive_torque[i]) * (1-self.veh_config["cDiff"])
            self.driver_input.wheel_torque_Nm.rear_left = (0.5 * self.drive_torque[i]) * self.veh_config["cDiff"]
            self.driver_input.wheel_torque_Nm.rear_right = (0.5 * self.drive_torque[i]) * self.veh_config["cDiff"]
                   
        #calculate brake torque
        if self.drive_torque[i] < 0:
            brake_torque_front = self.veh_config["brake_balance"] * self.drive_torque[i] 
            brake_torque_rear = (1 - self.veh_config["brake_balance"]) * self.drive_torque[i] 
            self.driver_input.wheel_torque_Nm.front_left  = brake_torque_front
            self.driver_input.wheel_torque_Nm.front_right = brake_torque_front
            self.driver_input.wheel_torque_Nm.rear_left = brake_torque_rear
            self.driver_input.wheel_torque_Nm.rear_right = brake_torque_rear

        self.veh.set_driver_input(self.driver_input)

    def get_simstate(self, i):
        """
        Retrieves the current simulation state from the vehicle model and performs further calculations.

        This function gets the output from the vehicle model, including the velocity, acceleration, angular velocity, and
        lateral acceleration. It also calculates the vehicle's speed, sideslip angle, and lateral acceleration in the
        vehicle's velocity frame. Additionally, it filters some signals and checks if the speed controller has settled
        and external forces have been ramped up. If so, it starts the delta ramp.

        Parameters:
            i (int): The current simulation step index.

        """
        #get output from model
        self.out = self.veh.get_output()
        if i>0 and self.sim_config['debug_log']:
            self.debug = self.veh.get_debug_out()
            if not hasattr(self, 'debug_size'):
                self.debug_size = len(self.debug.get_values())
                self.debug_arr = np.zeros((self.sim_config['n_sim']-1, self.debug_size))
                self.debug_header = self.debug.get_signal_names()
                self.debug_arr[i-1,:] = self.debug.get_values()
            else:
                self.debug_arr[i-1,:] = self.debug.get_values()
        self.vx_mps[i] = self.out.velocity_mps.x
        self.vy_mps[i] = self.out.velocity_mps.y
        self.ay_mps2[i] = self.out.acceleration_mps2.y
        self.ax_mps2[i] = self.out.acceleration_mps2.x #+ self.ax_mps2_set
        self.dpsi_radps[i] = self.out.angular_velocity_radps.z
        #further calculations
        self.v_mps[i] = np.sqrt(np.square(self.vx_mps[i]) + np.square(self.vy_mps[i]))
        self.v_err[i]=self.v_mps[i]-self.v_set
        if self.v_mps[i] != 0:
            self.beta_rad[i] = np.arctan(self.vy_mps[i] / self.vx_mps[i])
            # convert ay to velocity frame
            self.ay_vf_mps2[i] = self.ay_mps2[i] * np.cos(self.beta_rad[i])
            self.dpsi_radps_ss[i]=self.ay_vf_mps2[i]/self.v_mps[i]
        # filter some signals 
        if i>self.n_mean:
            self.ax_mean_mps2[i]=np.mean(self.ax_mps2[i-self.n_mean:i])
            self.dpsi_mean_radps[i]=np.mean(self.dpsi_radps[i-self.n_mean:i])
            self.dpsi_mean_radps_ss[i]=np.mean(self.dpsi_radps_ss[i-self.n_mean:i])
            self.dpsi_radps_err[i]=np.abs(self.dpsi_mean_radps[i]-self.dpsi_mean_radps_ss[i])
            self.ddpsi_radps_err[i]=np.abs(((self.dpsi_mean_radps[i]-self.dpsi_mean_radps[i-1])
                                        -(self.dpsi_mean_radps_ss[i]-self.dpsi_mean_radps_ss[i-1]))/
                                        self.sim_config['t_s'])
            # wait for speed controller to be settled and external forces to be ramped up before starting the delta ramp
            if np.abs(self.v_err[i])<0.05 and np.abs(self.ax_mean_mps2[i])<0.1 and self.step_response_ok and self.start_delta_ramp==False and np.abs(self.ddpsi_radps_err[i])<self.sim_config['ddpsi_err_detect']:
                self.start_delta_ramp=True
                self.start_delta_ramp_i=i


    def analyze(self):

        """
        Analyze the simulation data to find the maximum stable lateral acceleration.

        This function analyzes the data from the simulation to find the maximum stable lateral acceleration of the vehicle.
        It does this by iterating over the simulation data, starting from the point where the delta ramp starts, and checks
        if the yaw rate error is below a certain threshold. If the yaw rate error is too high, it iterates backwards to find
        the beginning of the instability. If the yaw rate error is below the threshold, it picks the highest lateral
        acceleration found in the delta ramp period.

        Parameters:
            None

        Returns:
            None
        """
        if self.start_delta_ramp:
            #find max stable lateral acceleration
            for i in range(self.start_delta_ramp_i, self.sim_config['n_sim']-20):
                #abort if yawrate error is too high and lateral acc is higher then already seen
                #if self.dpsi_radps_err[i]>self.sim_config['ddpsi_err_detect'] and self.ay_vf_mps2[i]>self.ay_mps2_max:
                if self.dpsi_radps_err[i]>self.sim_config['ddpsi_err_detect']:
                    #iterate backwards to find beginning of instability
                    for j in range(i, self.start_delta_ramp,-1):
                        if self.ddpsi_radps_err[j]<self.sim_config['ddpsi_err_max']:
                            self.ay_mps2_max=np.max([self.ay_vf_mps2[j],0])
                            self.ol_unstable=True
                            break
                    break

                #else pick highest lat acc found in delta ramp period
                if self.ay_mps2_max<self.ay_vf_mps2[i]:
                    self.ay_mps2_max=self.ay_vf_mps2[i]
                    k=i
            
        else:
            self.ax_unfeasible=True
            self.ol_unstable=True
            self.ay_mps2_max=0

    def plot(self):
        """
        Plots the simulation results.

        This function plots the time series of the vehicle's velocity, lateral acceleration, yaw rate, and other relevant
        quantities. It also plots the maximum stable lateral acceleration and the delta rise rate.

        Note: This is a debug function and only used in the singlerun mode

        Parameters:
            None

        Returns:
            None
        """
        print(f" start delta ramp {self.start_delta_ramp} at t={self.time_vec[self.start_delta_ramp_i]}s")
        print(f" delta rise rate {self.sim_config['delta_rise_rate']}radps")
        print(f" open loop unstable {self.ol_unstable}")
        print(f" ax unfeasible {self.ax_unfeasible}")
        print(f" max lat acc {self.ay_mps2_max}")
        fig, axs = plt.subplots(4, 2, sharex=True)

        axs[0, 0].plot(self.time_vec, self.v_mps, label='v mps')
        axs[0, 0].plot(self.time_vec, self.vx_mps, label='vx mps')
        axs[0, 0].plot(self.time_vec, self.v_err, label='v error mps')
        axs[0, 0].set_title('vx')
        axs[0, 0].legend()
        axs[0, 1].plot(self.time_vec, self.ay_mps2, label='ay mps2')
        axs[0, 1].plot(self.time_vec, self.ay_vf_mps2, label='ay velocity frame mps2')
        axs[0, 1].plot(self.time_vec, self.ay_mps2_max*np.ones(self.sim_config['n_sim']))
        axs[0, 1].set_title('ay')
        axs[0, 1].legend()
        axs[1, 0].plot(self.time_vec, self.dpsi_radps, label='dPsi raw')
        axs[1, 0].plot(self.time_vec, self.dpsi_mean_radps, label='dPsi')
        axs[1, 0].plot(self.time_vec, self.dpsi_mean_radps_ss, label='dPsi ss')
        axs[1, 0].plot(self.time_vec, self.dpsi_radps_err, label='dPsi error')
        axs[1, 0].plot(self.time_vec, self.ddpsi_radps_err, label='ddPsi error')
        axs[1, 0].set_title('dPsi')
        axs[1, 0].legend()
        axs[1, 1].plot(self.time_vec, self.delta, 'tab:red')
        axs[1, 1].set_title('delta')
        axs[2, 0].plot(self.time_vec, self.f_ext_x, 'tab:green')
        axs[2, 0].set_title('External Fx')
        axs[2, 1].plot(self.time_vec, self.beta_rad , 'tab:cyan')
        axs[2, 1].set_title('beta')
        axs[3, 0].plot(self.time_vec, self.drive_torque, 'tab:purple')
        axs[3, 0].set_title('drive torque')
        axs[3, 1].plot(self.time_vec, self.ax_mps2, label='ax mps2')
        axs[3, 1].plot(self.time_vec, self.ax_mean_mps2, label='ax mean mps2')
        axs[3, 1].set_title('ax mean mps2')
        for ax in axs.flatten():
            ax.grid(which='both', alpha=0.2)
            ax.minorticks_on()
            ax.tick_params(bottom=True, labelbottom=True)
            ax.legend()
        plt.show()

    def save(self, folder):
        """
        Saves the simulation data to CSV files in the specified folder.

        This function saves the time series data of the simulation, including
        various dynamic parameters such as acceleration, velocity, yaw rate, 
        and more, into a CSV file named 'timeseries.csv'. If debugging is enabled,
        it also saves additional debug data into 'sim_debug.csv'.

        Parameters:
            folder (str): The directory path where the data should be saved.

        Returns:
            None
        """

        folder = folder + '/data'
        file = folder + '/timeseries.csv' 
        data = np.column_stack((self.time_vec, self.ax_mps2, self.ay_mps2, self.ay_vf_mps2, self.beta_rad, self.delta, 
                                self.dpsi_radps, self.drive_torque, self.v_mps, self.vx_mps, self.vy_mps))
        header = ['time_s', 'ax_mps2', 'ay_mps2', 'ay_velocity_frame_mps2', 'beta_rad', 'delta_rad', 'dpsi_rad', 'drive_torque_nm',
                    'v_mps', 'vx_mps', 'vy_mps']
        df = pd.DataFrame(data, columns=header)
        df.to_csv(file)
        
        if self.sim_config['debug_log']:
            file_debug = folder + '/sim_debug.csv'
            df = pd.DataFrame(self.debug_arr, columns=self.debug_header)
            df.to_csv(file_debug) 

    def results_return(self):
        """
        Collects and returns the results of the simulation.

        This function compiles various results from the simulation into a dictionary.
        It includes information about whether the longitudinal acceleration is 
        unfeasible, if the vehicle is open-loop unstable, the maximum lateral 
        acceleration achieved, and the lateral acceleration gradient.

        Returns:
            dict: A dictionary containing the following key-value pairs:
                - 'ax_unfeasible' (bool): Indicates if the longitudinal acceleration is unfeasible.
                - 'ol_unstable' (bool): Indicates if the vehicle is open-loop unstable.
                - 'ay_mps2_max' (float): The maximum stable lateral acceleration in m/s².
                - 'ay_gradient' (float): The gradient of the lateral acceleration.
        """

        results = {}
        results['ax_unfeasible']=self.ax_unfeasible
        results['ol_unstable']=self.ol_unstable
        results['ay_mps2_max']=self.ay_mps2_max
        results['ay_gradient']=self.ay_gradient
        return results