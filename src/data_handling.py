import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

"""
Created by: Frederik Werner
Created on: 02.03.2023
"""

class data_handler:

    def __init__(self, params, t_start):
        
        """
        Initializes the data_handler object with simulation parameters and arrays.

        This constructor extracts parameters from the provided dictionary and initializes
        various attributes related to simulation steps and ranges for vertical acceleration,
        velocity, and longitudinal acceleration. It also sets up storage arrays for use
        during the simulation.

        Args:
            params (dict): A dictionary containing configuration parameters, including 'gggv_config'.
            t_start (float): The start time for the simulation, used for timing calculations.

        Attributes:
            singlerun (bool): Indicates if the simulation is a single run.
            gt_steps (int): Number of steps for vertical acceleration.
            v_steps (int): Number of steps for velocity.
            ax_steps (int): Number of steps for longitudinal acceleration.
            gt_range (np.ndarray): Range of vertical acceleration values.
            v_range (np.ndarray): Range of velocity values.
            ax_range (np.ndarray): Range of longitudinal acceleration values.
            ax_zero_idx (int): Index of zero in ax_range.
            nLines (int): Number of lines for data arrays.
            n_total (int): Total number of simulation steps.
            t_step_last (float): Last recorded time step.
            gt_arr (np.ndarray): Array for vertical acceleration per line.
            v_arr (np.ndarray): Array for velocity per line.
            ax_arr (np.ndarray): Array for longitudinal acceleration per line.
            sim_idx_arr (range): Range object for simulation index.
            gg_array (np.ndarray): Array to store postprocessing results.
        """
        # extract params
        gggv_config = params['gggv_config']

        # sim information
        self.singlerun = False
        self.gt_steps = gggv_config["gt_steps"]
        self.v_steps = gggv_config["v_steps"]
        self.ax_steps = gggv_config["ax_steps"]
        self.gt_range = np.linspace(gggv_config["gt_min"], gggv_config["gt_max"], self.gt_steps)
        self.v_range = np.linspace(gggv_config["v_min"], gggv_config["v_max"], self.v_steps)
        self.ax_range = np.linspace(gggv_config["ax_min"], gggv_config["ax_max"], self.ax_steps)
        # if no 0 in ax range add
        for i in range(self.ax_steps):
            if self.ax_range[i] > 0:
                self.ax_range = np.insert(self.ax_range, i, 0)
                self.ax_steps += 1
                self.ax_zero_idx = i
                break
            if self.ax_range[i] == 0:
                self.ax_zero_idx = i
                break
        # sim information
        self.nLines = self.ax_steps * self.v_steps
        self.n_total = self.gt_steps * self.v_steps * self.ax_steps
        self.t_step_last = t_start
        # storage arrays
        self.gt_arr = np.zeros(self.nLines, dtype='float64')
        self.v_arr = np.zeros(self.nLines, dtype='float64')
        self.ax_arr = np.zeros(self.nLines, dtype='float64')
        self.sim_idx_arr = range(self.nLines)
        # postprocessing arrays
        self.gg_array = np.zeros((self.gt_steps, self.v_steps, self.ax_steps, 5))

    def config_single(self, gt, v, ax):
        """
        Configures the simulation for a single run with specified parameters.

        Args:
            gt (float): The target vertical acceleration for the single run.
            v (float): The target velocity for the single run.
            ax (float): The target longitudinal acceleration for the single run.

        Sets:
            self.singlerun (bool): Flag to indicate single-run mode.
            self.gt_set_single (float): Stores the single run's target vertical acceleration.
            self.v_set_single (float): Stores the single run's target velocity.
            self.ax_set_single (float): Stores the single run's target longitudinal acceleration.
        """

        self.singlerun = True
        self.gt_set_single = gt
        self.v_set_single = v
        self.ax_set_single = ax
        
    def allocate(self, gt):
        """
        Documentation
        This function generates the input vector for the simulation. Each of the 4 input values (ax, vx, beta, delta)
        will have a dedicated vector which will be passed to the simulation process pool executor

        Input:
        gggv_config:  dictionary containing the configuration regarding gggv layout
        plot_config:   dictionary containing the configuration regarding plot layout

        Output
        ax_arr:         input array for the simulation for the requested longitudinal acceleration [m/s²]
        vx_arr:         input array for the simulation for the requested longitudinal velocity [m/s]
        ax_range:       ax_arr without duplicates
        vx_range:       vx_arr without duplicates
        beta_range:     beta_arr without duplicates
        delta_range:    delta_arr without duplicates
        """

        # get step count for input variable
        if self.singlerun == False:

            i = 0
            for v in range(self.v_steps):
                for ax in range(self.ax_steps):
                    self.gt_arr[i] = self.gt_range[gt]
                    self.v_arr[i] = self.v_range[v]
                    self.ax_arr[i] = self.ax_range[ax]
                    i = i + 1

        else:
            self.gt_steps = 1
            self.v_steps = 1
            self.ax_steps = 1
            self.gt_arr = self.gt_set_single
            self.v_arr = self.v_set_single
            self.ax_arr = self.ax_set_single
            self.nLines = 1
            self.sim_idx_arr = 1              
            
    def restructure(self, gt):
        if self.singlerun == False:
            for i in range(self.nLines):
                ax = np.where(self.ax_range == self.ax_arr[i])
                v = np.where(self.v_range == self.v_arr[i])
                # raw gg array
                self.gg_array[gt, v, ax, 0] = self.results_array[i]['ay_mps2_max']
                self.gg_array[gt, v, ax, 1] = self.ax_range[ax]
                self.gg_array[gt, v, ax, 2] = self.results_array[i]['ax_unfeasible']
                self.gg_array[gt, v, ax, 3] = self.results_array[i]['ol_unstable']
                self.gg_array[gt, v, ax, 4] = self.results_array[i]['ay_gradient']
                    
    def postprocessing(self, gt):
        if self.singlerun == False:
            # filter gg array
            self.mask_ax_unfeasible = self.gg_array[:, :, :, 2] == False
            self.mask_ol_unstable = self.gg_array[:, :, :, 3] == True

        else: 
            i = 0
            j = 0
            k = 0
            self.gg_array[i, j, k, 0] = self.results_array[i]['ay_mps2_max']
            self.gg_array[i, j, k, 0] = self.ax_range[0]
            self.gg_array[i, j, k, 0] = self.results_array[i]['ax_unfeasible']
            self.gg_array[i, j, k, 0] = self.results_array[i]['ol_unstable']
            self.gg_array[i, j, k, 0] = self.results_array[i]['ay_gradient']


    def plot(self, mirror, folder):

        """
        Plots the acceleration limits of the vehicle. 

        Args:
            mirror (bool): If True, the acceleration limits for negative lateral acceleration values are mirrored.
            folder (str): The folder where to save the plot.

        Returns:
            None
        """
        self.mirror_gg = mirror

        # Create subplot for raw
        fig1, ax1 = plt.subplots()
        plt.subplots_adjust(bottom=0.35)     

        # The parametrized function to be plotted
        l1 = [None] * self.v_steps
        for v in range(self.v_steps):
            if self.mirror_gg:
                x = self.gg_array[(self.gt_steps - 1), v, self.mask_ax_unfeasible[(self.gt_steps - 1), v, :], 0]
                y = self.gg_array[(self.gt_steps - 1), v, self.mask_ax_unfeasible[(self.gt_steps - 1), v, :], 1]
                x_mirr = np.concatenate(((-1 * x[::-1]), x))
                y_mirr = np.concatenate((y[::-1], y))
                # connect last to first point
                x_mirr = np.append(x_mirr, x_mirr[0])
                y_mirr = np.append(y_mirr, y_mirr[0])
                l1[v], = ax1.plot(x_mirr, y_mirr, label=f"{self.v_range[v]} mps")
            else:
                l1[v], = ax1.plot(self.gg_array[(self.gt_steps - 1), v, :, 0], self.gg_array[(self.gt_steps - 1), v, :, 1], label=f"{self.v_range[v]} mps")

        ax1.legend()
        ax1.grid()
        ax1.set_xlabel("lat. acceleration in mps2")
        ax1.set_ylabel("long. acceleration in mps2")
        ax1.set_title("Acceleration Limits")
        ax1.axis('equal')

        # Create axes for gt slider
        axgt1 = plt.axes([0.25, 0.15, 0.65, 0.03])
        
        # Create a slider from 0.0 to 20.0 in axes axfreq with 3 as initial value
        gt_select1 = Slider(axgt1, 'gt', np.min(self.gt_range), np.max(self.gt_range), np.min(self.gt_range), valstep=(self.gt_range[1] - self.gt_range[0]))

        # Create function to be called when slider value is changed        
        def update(val):
            gt = np.where(gt_select1.val == self.gt_range)[0][0]
            for v in range(self.v_steps):
                if self.mirror_gg:
                    x = self.gg_array[gt, v, self.mask_ax_unfeasible[gt, v, :], 0]
                    x_mirr = np.concatenate(((-1 * x[::-1]), x))
                    y = self.gg_array[gt, v, self.mask_ax_unfeasible[gt, v, :], 1]
                    y_mirr = np.concatenate((y[::-1], y))
                    # connect last to first point
                    x_mirr = np.append(x_mirr, x_mirr[0])
                    y_mirr = np.append(y_mirr, y_mirr[0])
                    l1[v].set_xdata(x_mirr)
                    l1[v].set_ydata(y_mirr)
                else:
                    l1[v].set_xdata(self.gg_array_filt[gt, v, :, 0])

        # Call update function when slider value is changed
        gt_select1.on_changed(update)
        gt_select1.set_val(np.min(self.gt_range))

        # save gg array
        name = '/plots/gggv'
        saveas = folder + name
        plt.savefig(saveas)
        
        plt.show()

    def runtime_est(self, t_step, gt):
        """
        Estimates the remaining simulation time.

        Args:
            t_step (float): The current time.
            gt (int): The current iteration step.

        Returns:
            time_left (float): The estimated remaining time in minutes.
        """
        self.n_act = (gt + 1) * self.nLines
        wait_time = t_step - self.t_step_last
        avg_time_per_sim = wait_time / self.nLines
        self.sims_left = self.n_total - self.n_act
        time_left = np.round(self.sims_left * avg_time_per_sim / 60)  # time in min
        self.t_step_last = t_step
        return time_left
        

    def save(self, folder):

        """
        Saves the simulation results and configuration arrays to files in the specified folder.

        This function saves the gg_array, which contains the simulation results, and the 
        velocity and vertical acceleration ranges (v_range and gt_range) to separate files 
        in the directory specified by the folder parameter. The files are saved as 
        .npy files for efficient storage and retrieval.

        Parameters:
            folder (str): The directory path where the data should be saved. The data 
            is stored in a subdirectory named 'data' within the specified folder.

        Returns:
            None
        """

        folder = folder + '/data'
        # save gg array
        name = '/gg_array'
        saveas = folder + name
        np.save(saveas, self.gg_array)

        # save lists
        name = '/v_list'
        saveas = folder + name
        np.save(saveas, self.v_range)
        name = '/g_list'
        saveas = folder + name
        np.save(saveas, self.gt_range)
        















        