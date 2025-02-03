import numpy as np
import src
import os
import datetime
import time
import concurrent.futures

"""
Created by: Frederik Werner
Created on: 28.02.2024
"""

# # -------------------------------------------------------------------------------------------------------------------
# USER INPUT ----------------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# assign valid param files
gggv_config_file = 'gggv_default'
sim_config_file = 'sim_default'
veh_config_file = 'veh_default'

#Simulate using all available cores
multi_threaded_execution=True

# input for single run only
singlerun=False
gt =9.0
v = 40.0
ax = 0.0

# # -------------------------------------------------------------------------------------------------------------------
# # Initialization ---------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# start time counter
t_start = time.perf_counter()

# create directories
folder = os.path.join(os.getcwd(), 'output', datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
src.directories.create(folder)

# assemble path to param files
path_root2module = os.path.dirname(os.path.abspath(__file__))
path_params = os.path.join(path_root2module, 'params/')

# load params 
params = src.sim_handling.param_load(gggv_config_file, sim_config_file, veh_config_file, folder, path_params)

# create sim handler
sim_handler=src.sim_handling.sim_handler(params)

# create data handler
data_handler = src.data_handling.data_handler(params, t_start)

#use single threaded execution for single runs
if singlerun:
    multi_threaded_execution = False
    debug_plot = True
else:
    debug_plot = False

print("Let's go!")

# # -------------------------------------------------------------------------------------------------------------------
# # Simulation----- -------------------- -------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------


# simulate model with constant steering angle and force request
def simulate(gt_set, v_set, ax_set, sim_idx):
    # local sim_handler
    sim = src.sim_handling.sim_handler(params)
    sim.set_parameter(v_set, ax_set, gt_set)
    sim.init_result_arrays()

    # initialize controller
    velocity_PID = src.PID.PID(sim.sim_config["velPIDparams"])

    # update print
    if not singlerun:
        gt_idx=np.where(data_handler.gt_range == gt_set)[0][0]
        n_act=gt_idx*data_handler.nLines+(sim_idx+1)
        print(f"starting sim {n_act}/{data_handler.n_total}: gt={np.round(gt_set, 2)}mps2, v={np.round(v_set, 2)}mps, ax={np.round(ax_set,2)}mps2")

    for i in range(sim.sim_config["n_sim"]):

        # get current vehicle dynamic state
        sim.get_simstate(i)

        #set external forces 
        if i > sim.n_wait_f_ext:
            sim.set_ext_f(i)
       
        if i>sim.n_wait_PID:
            #calculate drive torque
            if sim.drive:
                sim.drive_torque[i]= velocity_PID.run(v_set, sim.v_mps[i])
                sim.set_driver_input(i)

            # check if controller cannot keep target speed
            speed_tracking_error = np.abs(sim.v_err[i])>5
            if speed_tracking_error:
                if singlerun:
                    print('Loop break because speed tracking error')
                break

        if sim.start_delta_ramp:
            # abort if vehicle is open-loop unstable or ay is not increasing anmyore 
            if 'ay_curr' not in locals():
                ay_curr = sim.ay_mps2[i]
                j = 0
            ay_old = ay_curr
            ay_curr = sim.ay_mps2[i]
            if ay_curr > ay_old:
                j = 0
            else: 
                j += 1
            ol_unstable = np.abs(sim.ddpsi_radps_err[i])>sim.sim_config['ddpsi_err_detect']
            ay_max_found = j > (2/sim.sim_config["t_s"])
            
            if ol_unstable:
                if singlerun:
                    print('Loop break because unstable')
                break
            if ay_max_found:
                if singlerun:
                    print('Loop break because ay max found')
                break

            if sim.delta[i]>sim.sim_config["delta_max"]:
                if singlerun:
                    print('Loop break because delta max reached')
                break
            
        # simulate
        sim.veh.step()
        
    # Find max open-loop lateral acceleration
    sim.analyze()
    results_return=sim.results_return()


    # Plot
    if debug_plot:
        # Print running time
        t_stop = time.perf_counter()
        print("simulation time: " + str(t_stop - t_start) + "s")
        #plot and save
        sim.plot()
        sim.save(folder)

    # deconstruct 
    del sim
    del velocity_PID

    return results_return

# # simulation loop
if not singlerun:
    for gt in range(data_handler.gt_steps):
        # create sim array
        data_handler.allocate(gt)
        t_step = time.perf_counter()
        if multi_threaded_execution:
            # simulate using all available processor cores via process pool executor
            data_handler.results_array = [None]*data_handler.nLines
            i=0
            with concurrent.futures.ProcessPoolExecutor() as executor:
                for results_executor in executor.map(simulate, data_handler.gt_arr, data_handler.v_arr, data_handler.ax_arr, data_handler.sim_idx_arr):
                    data_handler.results_array[i] = results_executor
                    i=i+1
            t_step = time.perf_counter()
            time_left=data_handler.runtime_est(t_step,gt)
            print(f'Time remaining: {time_left} min. Clearing cache...')
            data_handler.restructure(gt)
            data_handler.postprocessing(gt)
        else:
            data_handler.config_single(gt, v, ax)
            data_handler.results_array=[None]*data_handler.nLines
            for i in range(data_handler.nLines):
                data_handler.results_array[i] = simulate(data_handler.gt_arr[i], data_handler.v_arr[i], data_handler.ax_arr[i], data_handler.sim_idx_arr[i])
            t_step = time.perf_counter()
            time_left=data_handler.runtime_est(t_step,gt)
            print(f'Time remaining: {time_left} min. Clearing cache...')                
            data_handler.postprocessing(gt)
else:
    data_handler.results_array = simulate(gt, v, ax, 0)

# Print running time
if not debug_plot:
    t_stop = time.perf_counter()
    print("simulation time: " + str(t_stop - t_start) + "s")
print("... hang on just a sec: post processing...")
t_start = time.perf_counter()

# # -------------------------------------------------------------------------------------------------------------------
# # Post Processing ---------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# save output file
data_handler.save(folder)

# Print running time
t_stop = time.perf_counter()
print("post processing time: " + str(t_stop - t_start) + "s")

if not singlerun:
    data_handler.plot(True, folder)


