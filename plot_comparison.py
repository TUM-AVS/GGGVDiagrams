import os
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from customtkinter import filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import sys

"""
Created by: Frederik Werner
Created on: 02.03.2023
"""

# ------------------------------------------------------------------------------
def instances():

    # Create the main window
    """
    Opens a GUI window to select the number of simulations for comparison.

    This function creates a tkinter window with a label and a spinbox to allow the user 
    to choose how many simulations they want to compare. The default value is set to 2.
    The user input is stored in a global variable `num_simulations` after pressing the OK button,
    and the window is closed.

    Note:
        - The number of simulations is limited from 1 to 10.
        - The selected number is printed for further processing.
    """

    root = tk.Tk()

    # Set the title of the window
    root.title("Number of simulations")

    # Create a label with the question
    label = ttk.Label(root, text="How many simulations do you want to compare?")

    # Create a spinbox for integer values with a default value of 2
    default_value = tk.StringVar(value="2")
    spinbox = ttk.Spinbox(root, from_=1, to=10, textvariable=default_value)

    # Create a function to be called when the user presses the OK button
    def ok_button_pressed():
        # Access the global variable
        global num_simulations
        # Get the value from the spinbox and write it to the global variable
        num_simulations = int(spinbox.get())
        # Close the window
        root.destroy()

    # Create the OK button
    ok_button = ttk.Button(root, text="OK", command=ok_button_pressed)

    # Pack the widgets in the main window
    label.pack()
    spinbox.pack()
    ok_button.pack()

    # Start the main window loop
    root.mainloop()

    # Use the global variable for further processing
    print(f"You want to compare {num_simulations} simulations.")


def cload():
    """
    Loads simulation data from a user-selected directory.

    This function opens a GUI dialog for the user to select a directory containing simulation output.
    It attempts to load the 'data' folder within the chosen directory, and extracts all .npy files
    into a dictionary where each key is the filename (without the extension) and the value is the 
    loaded numpy array.

    Returns:
        data_dict (dict): A dictionary containing the data from all .npy files in the selected directory.

    Raises:
        SystemExit: If the directory selection fails after a maximum number of attempts.
    """

    rel_path = os.path.join("gggv_sim", "output")

    # Absolute path to the desired folder
    abs_path = os.path.join(os.getcwd(), rel_path)

    root = ctk.CTk()
    root.withdraw()

    # Attempt to select the directory
    max_attempts = 3
    attempts = 0
    while attempts < max_attempts:
        try:
            load_path = os.path.join(filedialog.askdirectory(initialdir=abs_path, title="Select the directory and open it"), 'data')
            
            # Create a list of all files in the path
            file_list = os.listdir(load_path)
            break
        except:
            # Show error message and ask user to try again
            print("Error: Could not open the directory. Please select the desired directory with a double-click and confirm with 'OK'.")
            attempts += 1

            # If the maximum number of attempts is reached, exit the program with an error message
            if attempts == max_attempts:
                print("Error: Maximum number of attempts reached. Program will exit.")
                sys.exit()

    # Confirm successful loading
    print("Directory loaded successfully")
    root.destroy() 

    # Filter out all files that are not .npy files
    npy_files = [f for f in file_list if f.endswith('.npy')]

    # Create an empty dictionary to store the data
    data_dict = {}

    # Load each file in the list with np.load()
    for file in npy_files:
        file_path = os.path.join(load_path, file)
        data = np.load(file_path)
        data_dict[file.split(".")[0]] = data

   
    return data_dict

def check(data):
    #Check if all required files are present in the given data dictionary.
    """
    Checks if all required files are present in the given data dictionary.

    Args:
        data (dict): The dictionary containing the data.

    Returns:
        dict: The same dictionary with some values rounded to 2 decimal places.
    """
    try:
        test=data["v_list"]
        test=data["g_list"]
        test=data["gg_array"]
    except:
        print("Error: Directory does not contain all required files. Program will exit.")
        sys.exit()

    data["v_list"] = np.round(data["v_list"], 2)
    data["g_list"] = np.round(data["g_list"], 2)
    data["v_steps"] = np.arange(0, len(data["v_list"]), 1, dtype=int)
        
    return data 

def update_picks(data, root, vars):
    """
    Updates the v_picks list in the given data dictionary with the indices of the checked velocities and quits the root window.

    Args:
        data (dict): The dictionary containing the data.
        root (tkinter.Tk): The root window.
        vars (list): A list of tkinter.BooleanVar objects.
    """
    data["v_picks"] = [i for i, var in enumerate(vars) if var.get()]
    root.quit()


def cselect_entries(data):
    """
    Opens a GUI dialog for the user to select velocities from a list.
    The selected velocities are stored in the "v_picks" key of the given data dictionary.

    Args:
        data (dict): The dictionary containing the data.

    Returns:
        dict: The same dictionary with the "v_picks" key updated.
    """
    root = ctk.CTk()
    root.title("Chart configurator")
    label = ctk.CTkLabel(root, text="Select velocities to display in mps:")
    label.pack(padx=10, pady=10, anchor="w")

    vars = []
    for i in range(len(data["v_list"])):
        var = ctk.BooleanVar()
        vars.append(var)
        cb = ctk.CTkCheckBox(root, text=data["v_list"][i], variable=var)
        cb.pack()

    btn_confirm = ctk.CTkButton(root, text="Confirm", command=lambda: update_picks(data, root, vars))
    btn_confirm.pack()

    root.mainloop()
    root.destroy()
    print(data["v_picks"])

    return data

def plot():
    # Create figure
    """
    Plots the acceleration limits from the given data dictionaries.

    The function creates a plot with subplots for each data dictionary. Each subplot
    shows the acceleration limits for the velocities selected by the user. The x-axis
    shows the lateral acceleration in mps2, the y-axis shows the longitudinal acceleration
    in mps2. A slider is added to each subplot to select the gear to be displayed.

    """
    fig1, ax1 = plt.subplots()
    plt.subplots_adjust(bottom=0.35)    
    farben =  ['blue', 'green', 'red', 'cyan', 'magenta',  
             'orange', 'purple', 'pink', 'brown', 'gray', 'olive', 
          'navy', 'teal', 'coral', 'gold', 'lime', 'maroon', 'orchid', 
          'peru', 'salmon', 'sienna', 'tan', 'thistle']
    linestyles = ['solid', 'dashed', 'dashdot', 'dotted']

    #Allocating axes:
    l = [] 
    for i, data in enumerate(data_dict):
        max_len = max(data["v_picks"])+1  
        row = [None] * max_len  
        l.append(row)

    # Create axes for gt slider
    axgt1 = plt.axes([0.25, 0.15, 0.65, 0.03])
    
    # Create a slider 
    gt_select1 = Slider(axgt1, 'gt', np.min(data["g_list"]), np.max(data["g_list"]), np.min(data["g_list"]), valstep=(data["g_list"][1]-data["g_list"][0]))

    # Iterate over Data Dictionaries
    for i, data in enumerate(data_dict):

        #Create mask
        mask_ax_unfeasible = data["gg_array"][:,:, :, 2] == False

        # The parametrized function to be plotted
        for v in data["v_picks"]:
            x = data["gg_array"][0, v, mask_ax_unfeasible[0,v,:], 0]
            y = data["gg_array"][0, v, mask_ax_unfeasible[0,v,:], 1]
            x_mirr = np.concatenate(((-1*x[::-1]), x))
            y_mirr = np.concatenate((y[::-1], y))
            #connect last to first point
            x_mirr = np.append(x_mirr, x_mirr[0])
            y_mirr = np.append(y_mirr, y_mirr[0])
            l[i][v], = ax1.plot(x_mirr, y_mirr,label=f"{data['v_list'][v]} mps", color=farben[v % len(farben)], linestyle=linestyles[i % len(linestyles)])


        ax1.legend()
        ax1.grid()
        ax1.set_xlabel("lat. acceleration in mps2")
        ax1.set_ylabel("long. acceleration in mps2")
        ax1.set_title("Acceleration Limits")
        ax1.axis('equal')



    # Create function to be called when slider value is changed        
    def update(val):
        # Iterate over Data Dictionaries
        for i, data in enumerate(data_dict):
            g = np.where(gt_select1.val == data["g_list"])[0][0]
            for v in data["v_picks"]:
                x = data["gg_array"][g,v,mask_ax_unfeasible[g,v,:],0]
                x_mirr = np.concatenate(((-1*x[::-1]), x))
                y = data["gg_array"][g,mask_ax_unfeasible[g,v,:],1]
                y_mirr = np.concatenate((y[::-1], y))
                #connect last to first point
                x_mirr = np.append(x_mirr, x_mirr[0])
                y_mirr = np.append(y_mirr, y_mirr[0])
                l[i][v].set_xdata(x_mirr)
                l[i][v].set_ydata(y_mirr)

    # Call update function when slider value is changed
    gt_select1.on_changed(update)
    gt_select1.set_val(np.min(data["g_list"]))

    plt.show()


if __name__ == "__main__":
    #check how many instances should get compared
    instances()
    #load data
    data_dict=[None]*num_simulations
    for i in range(num_simulations):
        data_dict[i]=cload()
        data_dict[i]=check(data_dict[i])
        data_dict[i]=cselect_entries(data_dict[i])
    
    #Generate Plot
    plot()
    sys.exit()



