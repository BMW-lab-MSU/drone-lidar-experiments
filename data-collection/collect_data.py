import sys
import multiprocessing
import tomllib
import argparse
import pandas as pd
import time
import h5py
import os
import numpy as np
import matplotlib.pyplot as plt
from rich.progress import track
from scipy.fft import fft

import motor_control.motor_control as motor_control
from quickset_pan_tilt import controller, protocol
# import wingbeat_lidar as lidar
from wingbeat_lidar.digitizer import Digitizer
import wingbeat_lidar.range_calibration as rangecal

# Initialize digitizer
def setup_digitizer(config_file="./config/adc-config.toml"):
    digitizer = Digitizer(config_file)
    digitizer.initialize()
    digitizer.configure()

    return digitizer

# Initialize pan-tilt mount controller
def setup_pan_tilt_controller(port):
    pan_tilt = controller.ControllerSerial(protocol.PTCR20(), port)

    # Make sure the pan-tilt mount is at (0,0) before starting; this
    # isn't very important, but we might as well start at a known location.
    pan_tilt.home()

    return pan_tilt

# Initialize drone controller
def setup_drone_controller(port):
    motor_control.connect(port)
    motor_control.arm()
    motor_control.set_throttle([0,0,0,0])

# Start threads for collecting rpm data from the drone
def setup_rpm_collection_process():
    # Setup multiprocessing
    collect_rpm = multiprocessing.Event()
    experiment_active = multiprocessing.Event()
    telemetry_stable = multiprocessing.Event()
    rpm_recv_pipe, rpm_send_pipe = multiprocessing.Pipe(False)
    rpm_collection_process = multiprocessing.Process(
        target=motor_control.collect_rpm_data, args=(collect_rpm, experiment_active, telemetry_stable, rpm_send_pipe)
    )

    return collect_rpm, experiment_active, telemetry_stable, rpm_recv_pipe, rpm_collection_process

# load in the COM ports for the drone and pan tilt associated
def load_port_configuration(config_file="./config/serial-ports.toml"):
    with open(config_file, "rb") as f:
        config = tomllib.load(f)

    return config["pan_tilt_port"], config["drone_port"]

# Pause data collection for configuration of the drones
def is_manual_adjustment_needed(experiment_params, idx):
    # if motor configuration, prop size, or fill factor changed, then we need to pause
    # the code so we can manually adjust the experiment setup

    if idx > 0:
        current_params = experiment_params.iloc[idx]
        past_params = experiment_params.iloc[idx - 1]

        motor_config_changed = (
            current_params["motor configuration"] != past_params["motor configuration"]
        )
        prop_size_changed = current_params["prop size"] != past_params["prop size"]
        n_blades_changed = current_params["# blades"] != past_params["# blades"]
        fill_factor_changed = (
            current_params["fill factor"] != past_params["fill factor"]
        )

        if prop_size_changed:
            print(f"Prop size needs to be changed to {current_params['prop size']}")
        if motor_config_changed:
            print(
                f"Motor config needs to be changed to {current_params['motor configuration']}"
            )
        if fill_factor_changed:
            print(f"Fill factor needs to be changed to {current_params['fill factor']}")
        if n_blades_changed:
            print(
                f"Number of blades needs to be changed to {current_params['# blades']}"
            )

        if (
            motor_config_changed
            or prop_size_changed
            or n_blades_changed
            or fill_factor_changed
        ):
            return True
        else:
            return False
    else:
        return False

# Set the throttle to a value from the experiment parameters sheet
def set_throttle(experiment_params, idx):
    throttle_fl = np.nan_to_num(experiment_params.at[idx, "throttle front left"])
    throttle_fr = np.nan_to_num(experiment_params.at[idx, "throttle front right"])
    throttle_bl = np.nan_to_num(experiment_params.at[idx, "throttle back left"])
    throttle_br = np.nan_to_num(experiment_params.at[idx, "throttle back right"])

    motor_control.set_throttle(
        [
            throttle_bl,
            throttle_fl,
            throttle_br,
            throttle_fr,
        ],
        ramp_interval=0.25,
    )
    print(f"Drone Motor Throttles FR: {throttle_fr} FL: {throttle_fl} BR: {throttle_br} BL: {throttle_bl}")

# Check if a specific row has valid data or not to be overwritten
def does_row_have_data(experiment_params, idx):
    """Check if there is data in the spreadsheet at row idx.
    
    This code checks if, at the index point provided, 
    whether the spreadsheet has any average rpm values saved.
    
    Args:
        experiment_params:
            The experiment parameters are the column headers of all of the necessay
            setup information as well as the data collected. 
            The parameters include:{
                [motor configuration],
                [prop size],
                [# blades],
                [fill factor],
                [lens tube extension distance],
                [# images],
                [tilt angle],
                [throttle front right],
                [throttle front left], 
                [throttle back right],
                [throttle back left],
                [motor rpm front right],
                [motor rpm front left],
                [motor rpm back right],
                [motor rpm back left],
                [motor rpm front right std dev],
                [motor rpm front left std dev],
                [motor rpm back right std dev],
                [motor rpm back left std dev],
                [prop frequency front right],
                [prop frequency front left], 
                [prop frequency back right],
                [prop frequency back left],
                [prop frequency front right std dev], 
                [prop frequency front left std dev], 
                [prop frequency back right std dev],
                [prop frequency back left std dev], 
                [distance (m)], 
                [filename]
                }
        idx:
            The row index in the spreadsheet to access the data at for this function

    Returns:
        has_valid_data:
            Boolean indicating whether the row has rpm data saved in it.
    """
    has_data = False
    has_valid_data = True

    throttle_fr = experiment_params.at[idx, "throttle front right"]
    throttle_fl = experiment_params.at[idx, "throttle front left"]
    throttle_br = experiment_params.at[idx, "throttle back right"]
    throttle_bl = experiment_params.at[idx, "throttle back left"]

    rpm_fr = experiment_params.at[idx, "motor rpm front right"]
    rpm_fl = experiment_params.at[idx, "motor rpm front left"]
    rpm_br = experiment_params.at[idx, "motor rpm back right"]
    rpm_bl = experiment_params.at[idx, "motor rpm back left"]

    rpm_fr_std = experiment_params.at[idx, "motor rpm front right std dev"]
    rpm_fl_std = experiment_params.at[idx, "motor rpm front left std dev"]
    rpm_br_std = experiment_params.at[idx, "motor rpm back right std dev"]
    rpm_bl_std = experiment_params.at[idx, "motor rpm back left std dev"] 

    if (isinstance(rpm_fr,str) or 
        isinstance(rpm_fl,str) or 
        isinstance(rpm_br,str) or 
        isinstance(rpm_bl,str) or
        isinstance(rpm_fr_std,str) or
        isinstance(rpm_fl_std,str) or
        isinstance(rpm_br_std,str) or
        isinstance(rpm_bl_std,str)):
        has_data = True

    if has_data:
        # print(f"Data seen:\n{rpm_fr.split()}\n{rpm_fl.split()}\n{rpm_br.split()}\n{rpm_bl.split()}")
        rpm_fr_array = rpm_fr.replace("["," ").replace("]"," ").split()
        rpm_fl_array = rpm_fl.replace("["," ").replace("]"," ").split()
        rpm_br_array = rpm_br.replace("["," ").replace("]"," ").split()
        rpm_bl_array = rpm_bl.replace("["," ").replace("]"," ").split()
        
        rpm_fr_std_array = rpm_fr_std.replace("["," ").replace("]"," ").split()
        rpm_fl_std_array = rpm_fl_std.replace("["," ").replace("]"," ").split()
        rpm_br_std_array = rpm_br_std.replace("["," ").replace("]"," ").split()
        rpm_bl_std_array = rpm_bl_std.replace("["," ").replace("]"," ").split()
        
        for i in range(len(rpm_fr_array)):
            if((float(rpm_fr_array[i]) == 0 and 
                float(rpm_fl_array[i]) == 0 and 
                float(rpm_br_array[i]) == 0 and 
                float(rpm_bl_array[i]) == 0) or
               (float(rpm_fr_std_array[i]) == 0 and 
                float(rpm_fl_std_array[i]) == 0 and 
                float(rpm_br_std_array[i]) == 0 and 
                float(rpm_bl_std_array[i]) == 0)):
                has_valid_data = False
                if isinstance(throttle_fr,int):
                    if throttle_fr == 0:
                        has_valid_data = True
                    else:
                        has_valid_data = False
                if isinstance(throttle_fl,int):
                    if throttle_fl == 0:
                        has_valid_data = True   
                    else:
                        has_valid_data = False
                if isinstance(throttle_br,int):
                    if throttle_br == 0:
                        has_valid_data = True
                    else:
                        has_valid_data = False
                if isinstance(throttle_bl,int):
                    if throttle_bl == 0:
                        has_valid_data = True
                    else:
                        has_valid_data = False
    else:
        has_valid_data = False
    return has_valid_data 

# Record the motor data to the spreadsheet
def save_rpm_in_dataframe(experiment_params, idx, avg_rpm, std_dev_rpm):
    """Record motor data to spreadsheet at row idx

    This function records data to specific locations in the spreadsheet.
    The location is determined by the row index (idx), and the experiment
    parameters (experiment_params). Each experiment parameter that is previously
    determined to be data we are collecting is nominaally left empty and is 
    overwritten in this function if filled at the row index provided.
    
    Args:
        experiment_params:
            The experiment parameters are the column headers of all of the necessay
            setup information as well as the data collected. 
            The parameters include:{
                [motor configuration],
                [prop size],
                [# blades],
                [fill factor],
                [lens tube extension distance],
                [# images],
                [tilt angle],
                [throttle front right],
                [throttle front left], 
                [throttle back right],
                [throttle back left],
                [motor rpm front right],
                [motor rpm front left],
                [motor rpm back right],
                [motor rpm back left],
                [motor rpm front right std dev],
                [motor rpm front left std dev],
                [motor rpm back right std dev],
                [motor rpm back left std dev],
                [prop frequency front right],
                [prop frequency front left], 
                [prop frequency back right],
                [prop frequency back left],
                [prop frequency front right std dev], 
                [prop frequency front left std dev], 
                [prop frequency back right std dev],
                [prop frequency back left std dev], 
                [distance (m)], 
                [filename]
                }
        idx:
            The row index in the spreadsheet to access the data at for this function
        avg_rpm:
            This data contains the information recieved from the drone, serving as
            our ground truth data. This data contains the motor rotation speed
            information from all 4 motors attached to the drone.
            The motor configoration is as follows:{
                [Back Left],
                [Front Left],
                [Back Right],
                [Front Right]
            }
        std_dev_rpm:
            This data contains the standard deviation of the motor rotation speed
            information from all 4 motors attached to the drone.
            The motor configoration is as follows:{
                [Back Left],
                [Front Left],
                [Back Right],
                [Front Right]
            }
    """
    experiment_params.at[idx, "motor rpm front right"] = str(avg_rpm[:, 3])
    experiment_params.at[idx, "motor rpm front left"] = avg_rpm[:, 1]
    experiment_params.at[idx, "motor rpm back right"] = avg_rpm[:, 2]
    experiment_params.at[idx, "motor rpm back left"] = avg_rpm[:, 0]

    experiment_params.at[idx, "motor rpm front right std dev"] = std_dev_rpm[:, 3]
    experiment_params.at[idx, "motor rpm front left std dev"] = std_dev_rpm[:, 1]
    experiment_params.at[idx, "motor rpm back right std dev"] = std_dev_rpm[:, 2]
    experiment_params.at[idx, "motor rpm back left std dev"] = std_dev_rpm[:, 0]

    experiment_params.at[idx, "prop frequency front right"] = _compute_prop_frequency(
        avg_rpm[:, 3], experiment_params, idx
    )
    experiment_params.at[idx, "prop frequency front left"] = _compute_prop_frequency(
        avg_rpm[:, 1], experiment_params, idx
    )
    experiment_params.at[idx, "prop frequency back right"] = _compute_prop_frequency(
        avg_rpm[:, 2], experiment_params, idx
    )
    experiment_params.at[idx, "prop frequency back left"] = _compute_prop_frequency(
        avg_rpm[:, 0], experiment_params, idx
    )

    experiment_params.at[idx, "prop frequency front right std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 3], experiment_params, idx)
    )
    experiment_params.at[idx, "prop frequency front left std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 1], experiment_params, idx)
    )
    experiment_params.at[idx, "prop frequency back right std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 2], experiment_params, idx)
    )
    experiment_params.at[idx, "prop frequency back left std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 0], experiment_params, idx)
    )

# calculate the propeller frequency from the number of props and motor RPM
def _compute_prop_frequency(motor_rpm, experiment_params, idx):
    """Covert motor rpm into propeller frequency.

    This function converts the motor rpm into the propeller frequency as seen
    by the lidar:

    frequency = motor_rpm * (# of propeller blades) / 60 [s]

    Strictly speaking, this is not the frequency of a *single propeller blade*,
    as that would just be the motor_rpm / 60. This is the frequency at which
    any propeller blade passes through a particular location; as such, during
    one revolution, the number of blades that pass through a particular
    location is just the number of blades on the propeller.

    Args:
        experiment_params:
            The experiment parameters are the column headers of all of the necessay
            setup information as well as the data collected. 
            The parameters include:{
                [motor configuration],
                [prop size],
                [# blades],
                [fill factor],
                [lens tube extension distance],
                [# images],
                [tilt angle],
                [throttle front right],
                [throttle front left], 
                [throttle back right],
                [throttle back left],
                [motor rpm front right],
                [motor rpm front left],
                [motor rpm back right],
                [motor rpm back left],
                [motor rpm front right std dev],
                [motor rpm front left std dev],
                [motor rpm back right std dev],
                [motor rpm back left std dev],
                [prop frequency front right],
                [prop frequency front left], 
                [prop frequency back right],
                [prop frequency back left],
                [prop frequency front right std dev], 
                [prop frequency front left std dev], 
                [prop frequency back right std dev],
                [prop frequency back left std dev], 
                [distance (m)], 
                [filename]
                }
        idx:
            The row index in the spreadsheet to access the data at for this function
        motor_rpm:
            Drone motor rotations per minute information to be converted into propeller frequency
    """
    n_blades = experiment_params.at[idx, "# blades"]

    # Conversion for revolutions per minute to revolutions per second
    RPM_TO_HZ = 1 / 60

    prop_frequency = motor_rpm * n_blades * RPM_TO_HZ

    return prop_frequency

# Set the tilt angle for the pan tilt controller holding the drones
def set_tilt_angle(pan_tilt, experiment_params, idx):
    # Check for faults and clear any that exist
    hard_faults, soft_faults = pan_tilt.check_for_faults(pan_tilt.get_status())
    # print(hard_faults)
    if hard_faults:
        pan_tilt.fault_reset()

    print(experiment_params.at[idx, "tilt angle"])
    pan_tilt.move_absolute(0, experiment_params.at[idx, "tilt angle"])

# Create the name of the h5 file for structured storage of data
def create_h5_filename(experiment_params, idx, filename_prefix):
    """Creates the filename for the h5 file
    
    Creates the name based on thr timestamp, tilt angle and drone parameters.

    Args:
        experiment_params:
            The experiment parameters are the column headers of all of the necessay
            setup information as well as the data collected. 
            The parameters include:{
                [motor configuration],
                [prop size],
                [# blades],
                [fill factor],
                [lens tube extension distance],
                [# images],
                [tilt angle],
                [throttle front right],
                [throttle front left], 
                [throttle back right],
                [throttle back left],
                [motor rpm front right],
                [motor rpm front left],
                [motor rpm back right],
                [motor rpm back left],
                [motor rpm front right std dev],
                [motor rpm front left std dev],
                [motor rpm back right std dev],
                [motor rpm back left std dev],
                [prop frequency front right],
                [prop frequency front left], 
                [prop frequency back right],
                [prop frequency back left],
                [prop frequency front right std dev], 
                [prop frequency front left std dev], 
                [prop frequency back right std dev],
                [prop frequency back left std dev], 
                [distance (m)], 
                [filename]
                }
        idx:
            The row index in the spreadsheet to access the data at for this function
        filename_prefix:
            An additional prefix able to be added by the user, but unnecessary
            for base filename creation

    """
    tilt_angle = f"tilt-{experiment_params.at[idx, 'tilt angle']}"

    throttle_fr = ""
    throttle_fl = ""
    throttle_br = ""
    throttle_bl = ""
    if isinstance(experiment_params.at[idx, 'throttle front right'], int):
        throttle_fr = f"-fr-{experiment_params.at[idx, 'throttle front right']}"
    if isinstance(experiment_params.at[idx, 'throttle front left'], int):
        throttle_fl = f"-fl-{experiment_params.at[idx, 'throttle front left']}"
    if isinstance(experiment_params.at[idx, 'throttle back right'], int):
        throttle_br = f"-br-{experiment_params.at[idx, 'throttle back right']}"
    if isinstance(experiment_params.at[idx, 'throttle back left'], int):
        throttle_bl = f"-bl-{experiment_params.at[idx, 'throttle back left']}"

    timestamp = f"{time.strftime('%H-%M-%S-')}"

    if filename_prefix is None:
        filename_prefix = ""
    else:
        filename_prefix = filename_prefix + "-"

    filename = (
        filename_prefix + timestamp + tilt_angle + throttle_fr + throttle_fl + throttle_br + throttle_bl
    )

    return filename

# Prompt the user for the length of the lens tube
def prompt_for_lens_tube_distance():
    """Prompts the user for the length of the lens tube
    
    The length of the lens tube affects the data due to the offset it creates
    in beam size as well as the divergance point.

    Returns:
        lens_tube_distance:
            The entered distance of how far the lens tube is extended.

    """
    is_lens_tube_distance_valid = False
    while not is_lens_tube_distance_valid:
        lens_tube_distance = input(
            "Enter the lens tube's extension distance: "
        )
        try:
            lens_tube_distance = float(lens_tube_distance)
            is_lens_tube_distance_valid = True
        except:
            print("Invalid lens tube distance. Please enter a number")

    return lens_tube_distance

# Save the h5 file with all of the data
def save_h5_file(h5_filename, data_dir, experiment_params, idx, 
                 data, timestamps, capture_time, avg_rpm, rpm_std_dev, 
                 digitizer, is_data_in_volts, distance):
    """Define the structure of the h5 file for experiment information

    This function creates the h5 file for storing all the information related to
    the experiment, including setup parameters, data collected, and data calculated.

    Args:
        h5_filename:
            The name for the h5 file for saving the file to memory.
        data_dir:
            The data directory for the file to be saved into.
        experiment_params:
            The experiment parameters are the column headers of all of the necessay
            setup information as well as the data collected. 
            The parameters include:{
                [motor configuration],
                [prop size],
                [# blades],
                [fill factor],
                [lens tube extension distance],
                [# images],
                [tilt angle],
                [throttle front right],
                [throttle front left], 
                [throttle back right],
                [throttle back left],
                [motor rpm front right],
                [motor rpm front left],
                [motor rpm back right],
                [motor rpm back left],
                [motor rpm front right std dev],
                [motor rpm front left std dev],
                [motor rpm back right std dev],
                [motor rpm back left std dev],
                [prop frequency front right],
                [prop frequency front left], 
                [prop frequency back right],
                [prop frequency back left],
                [prop frequency front right std dev], 
                [prop frequency front left std dev], 
                [prop frequency back right std dev],
                [prop frequency back left std dev], 
                [distance (m)], 
                [filename]
                }
        idx:
            The row index in the spreadsheet to access the data at for this function
        data:
             
        timestamps:

        capture_time:

        avg_rpm:
            This data contains the information recieved from the drone, serving as
            our ground truth data. This data contains the motor rotation speed
            information from all 4 motors attached to the drone.
            The motor configoration is as follows:{
                [Back Left],
                [Front Left],
                [Back Right],
                [Front Right]
            }
        std_dev_rpm:
            This data contains the standard deviation of the motor rotation speed
            information from all 4 motors attached to the drone.
            The motor configoration is as follows:{
                [Back Left],
                [Front Left],
                [Back Right],
                [Front Right]
            }
        digitizer:
            The digitizer object for referring to the data collected from it
        is_data_in_volts: 

        distance:
            How far away the drone is in meters.
    """
    os.makedirs(data_dir, exist_ok=True)

    with h5py.File(data_dir + os.sep + h5_filename + ".hdf5", "w") as h5file:
        digitizer.save_data_in_h5(h5file, data, timestamps, capture_time, is_data_in_volts, distance)

        h5file.create_group("parameters/motor_rpm/front_right")
        h5file.create_group("parameters/motor_rpm/front_left")
        h5file.create_group("parameters/motor_rpm/back_right")
        h5file.create_group("parameters/motor_rpm/back_left")

        h5file["parameters/motor_rpm/front_right/avg"] = avg_rpm[:,3]
        h5file["parameters/motor_rpm/front_right/std_dev"] = rpm_std_dev[:,3]
        h5file["parameters/motor_rpm/front_left/avg"] = avg_rpm[:,1]
        h5file["parameters/motor_rpm/front_left/std_dev"] = rpm_std_dev[:,1]
        h5file["parameters/motor_rpm/back_right/avg"] = avg_rpm[:,2]
        h5file["parameters/motor_rpm/back_right/std_dev"] = rpm_std_dev[:,2]
        h5file["parameters/motor_rpm/back_left/avg"] = avg_rpm[:,0]
        h5file["parameters/motor_rpm/back_left/std_dev"] = rpm_std_dev[:,0]

        h5file.create_group("parameters/prop_frequency/front_right")
        h5file.create_group("parameters/prop_frequency/front_left")
        h5file.create_group("parameters/prop_frequency/back_right")
        h5file.create_group("parameters/prop_frequency/back_left")

        h5file["parameters/prop_frequency/front_right/avg"] = _compute_prop_frequency(avg_rpm[:,3], experiment_params, idx)
        h5file["parameters/prop_frequency/front_right/std_dev"] =_compute_prop_frequency(rpm_std_dev[:,3], experiment_params, idx)
        h5file["parameters/prop_frequency/front_left/avg"] = _compute_prop_frequency(avg_rpm[:,1], experiment_params, idx)
        h5file["parameters/prop_frequency/front_left/std_dev"] = _compute_prop_frequency(rpm_std_dev[:,1], experiment_params, idx)
        h5file["parameters/prop_frequency/back_right/avg"] = _compute_prop_frequency(avg_rpm[:,2], experiment_params, idx)
        h5file["parameters/prop_frequency/back_right/std_dev"] = _compute_prop_frequency(rpm_std_dev[:,2], experiment_params, idx)
        h5file["parameters/prop_frequency/back_left/avg"] = _compute_prop_frequency(avg_rpm[:,0], experiment_params, idx)
        h5file["parameters/prop_frequency/back_left/std_dev"] = _compute_prop_frequency(rpm_std_dev[:,0], experiment_params, idx)

        h5file.create_group("parameters/throttle")

        throttle_fr = experiment_params.at[idx, "throttle front right"]
        throttle_fl = experiment_params.at[idx, "throttle front left"]
        throttle_br = experiment_params.at[idx, "throttle back right"]
        throttle_bl = experiment_params.at[idx, "throttle back left"]
        h5file["parameters/throttle/front_right"] = throttle_fr
        h5file["parameters/throttle/front_left"] = throttle_fl
        h5file["parameters/throttle/back_right"] = throttle_br
        h5file["parameters/throttle/back_left"] = throttle_bl

        h5file["parameters/tilt"] = experiment_params.at[idx, "tilt angle"]
        h5file["parameters/motor_configuration"] = experiment_params.at[idx, "motor configuration"]
        h5file["parameters/prop_size"] = experiment_params.at[idx, "prop size"]
        h5file["parameters/n_blades"] = experiment_params.at[idx, "# blades"]
        h5file["parameters/fill_factor"] = experiment_params.at[idx, "fill factor"]
        h5file["parameters/lens_tube_extension"] = experiment_params.at[idx, "lens tube extension distance"]
        h5file["parameters/target_distance"] = experiment_params.at[idx, "distance (m)"]

# Save a png of the image seen by the LIDAR
# TODO: Save a PNG of the frequency spectrum as well
def save_png(data, timestamps, filename, data_dir):
    """Save PNGs of the image seen by the lidar
    
    """
    plt.pcolormesh(data)

    plt.xticks(ticks=range(0,len(timestamps),256),labels=np.round(timestamps[0:2048:256]/1e6,1),rotation=45)
    plt.xlabel('time (ms)')

    # Invert the y axis so the top-left is (0,0), which is how we like to view the images
    # because that's how MATLAB does it :)
    # plt.gca().invert_yaxis()
    plt.ylabel('range bin')

    plt.savefig(data_dir + os.sep + filename + 'time-domain.png')
    plt.close()
    
def save_spectrograms(data, timestamps, filename, data_dir):
    
    rows = np.arange(115,124)
    for i in range(len(rows)):
        plt.subplot(int(len(rows)/3),int(len(rows)/3),i+1,)
        plt.title(f'range bin {rows[i]}')
        plt.xlabel('time (ms)')
        plt.ylabel('freqs')
        plt.specgram(data[rows[i],:],NFFT=256,noverlap=128,Fs=1*np.e**9,)
        plt.colorbar()

    plt.savefig(data_dir + os.sep + filename + 'frequency-domain.png')
    plt.close()
    

def main(
    experiment_spreadsheet_path,
    data_dir,
    digitizer_config,
    range_calibration_config,
    serial_port_config,
    filename_prefix,
    use_volts,
):
    N_MOTORS = 4

    experiment_params = pd.read_excel(experiment_spreadsheet_path, dtype=object)
    n_experiments = len(experiment_params)

    pan_tilt_port, drone_port = load_port_configuration(serial_port_config)

    print("--------------------------------------------")
    print("Setting up pan tilt mount:")
    pan_tilt = setup_pan_tilt_controller(pan_tilt_port)
    print("--------------------------------------------")


    print("Setting up drone motor control:")
    setup_drone_controller(drone_port)
    print("--------------------------------------------")
    collect_rpm, experiment_active, telemetry_stable, rpm_recv_pipe, rpm_collection_process = (
        setup_rpm_collection_process()
    )

    print("Setting up digitizer:")
    digitizer = setup_digitizer(digitizer_config)
    print("--------------------------------------------")

    # Read in ground-truth / experiment parameter spreadsheet so we have the
    # experiment parameters we need, i.e., motor speed, tilt angle, etc.

    # Get the image size that the digitizer is collecting so we can preallocate
    # matrices later for saving the data.
    n_samples = digitizer.acquisition_config.SegmentSize
    n_segments = digitizer.acquisition_config.SegmentCount

    # Spawn and start the process that collects rpm data from the drone.
    rpm_collection_process.start()

    # Tell the rpm collection process that it can run its main loop. The
    # process will won't collect any rpm data until collect_rpm is set.
    # However, we need this extra flag to tell the process to terminate
    # so we can join the process once everything is done; we need this flag
    # because we can't call the process's start method more than once; while
    # we could make a new process for every image, that would not be efficient.
    experiment_active.set()

    # motor_control.set_throttle([60,60,60,60])
    # time.sleep(20)


    lens_tube_distance = prompt_for_lens_tube_distance()            

    try:
        for idx in range(n_experiments):
            
            # Skip the current parameter set if the spreadsheet already has
            # data recorded for the current and next 2 rows.
            # has_data = [False, False, False]
            # for i in range(3):
            #     has_data[i] = does_row_have_data(experiment_params, idx+i)
            has_data = does_row_have_data(experiment_params,idx)
            if has_data:#[0] and has_data[1] and has_data[2]:
                print(f"Row {idx} has data, Continuing to row {idx + 1}")
                continue

            experiment_params.at[idx, "lens tube extension distance"] = lens_tube_distance

            if is_manual_adjustment_needed(experiment_params, idx):

                motor_control.set_throttle([0,0,0,0])

                lens_tube_distance = prompt_for_lens_tube_distance()            
                experiment_params.at[idx, "lens tube extension distance"] = lens_tube_distance

                answer = "n"
                while answer.lower() != "y":
                    answer = input(
                        'Press "y" when you are ready to run the next configuration: '
                    )

            print("--------------------------------------------")
            print(f"Setting throttles:")
            set_throttle(experiment_params, idx)
            print("--------------------------------------------")

            print("Setting tilt angle:")
            set_tilt_angle(pan_tilt, experiment_params, idx)
            print("--------------------------------------------")

            n_images = int(experiment_params.at[idx, "# images"])

            data = np.empty((n_images, n_samples, n_segments))
            timestamps = np.empty(shape=(n_images, n_segments))
            capture_time = np.empty(shape=n_images, dtype=np.bytes_)

            avg_rpm = np.empty((n_images, N_MOTORS))
            std_dev_rpm = np.empty((n_images, N_MOTORS))

            telemetry_stable.clear()

            print("Collecting data:")
            for image_num in range(n_images):

                # Tell the process to start collecting rpm telemetry
                # collect_rpm.set()

                # telemetry_stable.wait()

                # Collect the lidar data
                (
                    data[image_num, :, :],
                    timestamps[image_num, :],
                    capture_time[image_num],
                ) = digitizer.capture()

                # We're done collecting data, so stop collecting rpm telemetry
                # collect_rpm.clear()

                # Get the average rpm values
                (avg_rpm[image_num, :], std_dev_rpm[image_num, :]) = rpm_recv_pipe.recv()
                print_rpm = []
                for i in avg_rpm[image_num, :]: print_rpm.append(int(i))
                print_rpm = [print_rpm[3],print_rpm[1],print_rpm[2],print_rpm[0]]
                print(f"Image collected: {image_num},\tRPM: {print_rpm}")


            # Put the ground-truth rpm data into the dataframe

            save_rpm_in_dataframe(experiment_params, idx, avg_rpm, std_dev_rpm)

            if use_volts:
                data = digitizer.convert_to_volts(data)

            # If a range calibration file was given, convert range bins into meters
            if range_calibration_config:
                rangecal.load_calibration(range_calibration_config)

                range_bins = np.arange(n_samples)
                distance = rangecal.compute_range(range_bins)
            else:
                distance = None

            # Save the data, ground-truth, and metadata in an h5 file
            h5_filename = create_h5_filename(experiment_params, idx, filename_prefix)

            save_h5_file(
                h5_filename, data_dir, experiment_params, idx, data, timestamps, capture_time, avg_rpm, std_dev_rpm, digitizer, use_volts, distance
            )

            save_png(data[15,:,:], timestamps[15,:], h5_filename, data_dir)
            save_spectrograms(data[15,:,:], timestamps[15,:], h5_filename, data_dir) 
            # Put the data filename in the ground-truth dataframe
            experiment_params.at[idx, "filename"] = h5_filename

            # Save the spreadsheet just in case something in the experiment blows
            # up causing us to kill this code midway through the experiments.
            experiment_params.to_excel(experiment_spreadsheet_path, index=False)

        # The experiment is over; tell the rpm collection process that it can
        # terminate itself.
        experiment_active.clear()

        # Stop the rpm collection process
        rpm_collection_process.join()

        # save spreadsheet
        experiment_params.to_excel(experiment_spreadsheet_path, index=False)

        motor_control.set_throttle([0,0,0,0])

        pan_tilt.home()

    except KeyboardInterrupt:
        # The experiment is over; tell the rpm collection process that it can
        # terminate itself.
        experiment_active.clear()

        # Stop the rpm collection process
        rpm_collection_process.join()

        # save spreadsheet
        experiment_params.to_excel(experiment_spreadsheet_path, index=False)

        motor_control.set_throttle([0,0,0,0])

        pan_tilt.home()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "experiment_spreadsheet",
        type=str,
        help="Path to the experiment parameters spreadsheet",
    )
    parser.add_argument(
        "data_directory", type=str, help="Path to the top-level data directory"
    )
    parser.add_argument(
        "--digitizer-config",
        type=str,
        default="./config/digitizer.toml",
        help="Path to the digitizer configuration file. Default: ./config/adc-config.toml",
    )
    parser.add_argument(
        "--range-calibration-config",
        type=str,
        default=None,
        help="Path to the range calibration configuration file. Default: ./config/range-calibration.toml",
    )
    parser.add_argument(
        "--serial-port-config",
        type=str,
        default="./config/serial-ports.toml",
        help="Path to the serial port configuration file. Default: ./config/serial-ports.toml",
    )
    parser.add_argument(
        "--filename-prefix",
        type=str,
        default=None,
        help="Filename prefix for the data files.",
    )
    parser.add_argument(
        "--use-volts",
        action="store_true",
        help="Save the data in volts instead of raw ADC counts",
    )

    args = parser.parse_args()

    # TODO: argument validation

    sys.exit(
        main(
            args.experiment_spreadsheet,
            args.data_directory,
            args.digitizer_config,
            args.range_calibration_config,
            args.serial_port_config,
            args.filename_prefix,
            args.use_volts,
        )
    )
