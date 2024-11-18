import pandas as pd
import numpy as np
import argparse
import glob
import h5py
import sys
import os
import re

def extract_params(filename):
    match = re.search(r'tilt-(\d+)-([\d.]+)m-fr-(\d+)-fl-(\d+)-br-(\d+)-bl-(\d+)', filename)
    if match:
        return match.groups()
    return None

def combineH5(data_folder):
    '''
    Combine the data from the lidar and drone h5 files into a single h5 files
    '''
    # find all lidar h5 files in the data folder
    lidar_files = [file for file in glob.glob(data_folder + os.sep + "lidar*.hdf5")]

    for lidar_file in lidar_files:
        params = extract_params(lidar_file)
        if not params:
            continue

        # Find the corresponding drone file
        drone_file_pattern = f"drone*tilt-{params[0]}-{params[1]}m-fr-{params[2]}-fl-{params[3]}-br-{params[4]}-bl-{params[5]}.hdf5"
        drone_files = glob.glob(data_folder + os.sep + drone_file_pattern)
        if not drone_files:
            continue
        drone_file = drone_files[0]

        combined_params = {}

        combined_params["tilt"] = params[0]
        combined_params["target_distance"] = params[1]

        # Read lidar file
        with h5py.File(lidar_file, 'r') as data_file:
            combined_params["motor_configuration"] = data_file["parameters/motor_configuration"][()]
            combined_params["prop_size"] = data_file["parameters/prop_size"][()].decode('utf-8')
            combined_params["n_blades"] = data_file["parameters/n_blades"][()]
            combined_params["fill_factor"] = data_file["parameters/fill_factor"][()]
            combined_params["lens_tube_extension"] = data_file["parameters/lens_tube_extension"][()]
            combined_params["throttle"] = {}
            combined_params["throttle"]["front_right"] = data_file["parameters/throttle/front_right"][()]
            combined_params["throttle"]["front_left"] = data_file["parameters/throttle/front_left"][()]
            combined_params["throttle"]["back_right"] = data_file["parameters/throttle/back_right"][()]
            combined_params["throttle"]["back_left"] = data_file["parameters/throttle/back_left"][()]
            combined_params["data"] = {}
            combined_params["data"]["data"] = list(data_file["data/data"])
            combined_params["data"]["capture_time"] = list(data_file["data/capture_time"])
            combined_params["data"]["timestamps"] = list(data_file["data/timestamps"])
            combined_params["digitizer"] = {}
            combined_params["digitizer"]["config"] = {}
            combined_params["digitizer"]["config"]["acquisition"] = list(data_file["digitizer/config/acquisition"])
            combined_params["digitizer"]["config"]["trigger"] = list(data_file["digitizer/config/trigger"])
            combined_params["digitizer"]["config"]["channel"] = list(data_file["digitizer/config/channel"])
            combined_params["digitizer"]["info"] = list(data_file["digitizer/info"])

        # Read drone file
        with h5py.File(drone_file, 'r') as data_file:
            combined_params["motor_rpm"] = {}
            combined_params["motor_rpm"]["front_right"] = {}
            combined_params["motor_rpm"]["front_right"]["avg"] = list(data_file["parameters/motor_rpm/front_right/avg"])
            combined_params["motor_rpm"]["front_right"]["std_dev"] = list(data_file["parameters/motor_rpm/front_right/std_dev"])
            combined_params["motor_rpm"]["front_left"] = {}
            combined_params["motor_rpm"]["front_left"]["avg"] = list(data_file["parameters/motor_rpm/front_left/avg"])
            combined_params["motor_rpm"]["front_left"]["std_dev"] = list(data_file["parameters/motor_rpm/front_left/std_dev"])
            combined_params["motor_rpm"]["back_right"] = {}
            combined_params["motor_rpm"]["back_right"]["avg"] = list(data_file["parameters/motor_rpm/back_right/avg"])
            combined_params["motor_rpm"]["back_right"]["std_dev"] = list(data_file["parameters/motor_rpm/back_right/std_dev"])
            combined_params["motor_rpm"]["back_left"] = {}
            combined_params["motor_rpm"]["back_left"]["avg"] = list(data_file["parameters/motor_rpm/back_left/avg"])
            combined_params["motor_rpm"]["back_left"]["std_dev"] = list(data_file["parameters/motor_rpm/back_left/std_dev"])
            combined_params["prop_frequency"] = {}
            combined_params["prop_frequency"]["front_right"] = {}
            combined_params["prop_frequency"]["front_right"]["avg"] = list(data_file["parameters/prop_frequency/front_right/avg"])
            combined_params["prop_frequency"]["front_right"]["std_dev"] = list(data_file["parameters/prop_frequency/front_right/std_dev"])
            combined_params["prop_frequency"]["front_left"] = {}
            combined_params["prop_frequency"]["front_left"]["avg"] = list(data_file["parameters/prop_frequency/front_left/avg"])
            combined_params["prop_frequency"]["front_left"]["std_dev"] = list(data_file["parameters/prop_frequency/front_left/std_dev"])
            combined_params["prop_frequency"]["back_right"] = {}
            combined_params["prop_frequency"]["back_right"]["avg"] = list(data_file["parameters/prop_frequency/back_right/avg"])
            combined_params["prop_frequency"]["back_right"]["std_dev"] = list(data_file["parameters/prop_frequency/back_right/std_dev"])
            combined_params["prop_frequency"]["back_left"] = {}
            combined_params["prop_frequency"]["back_left"]["avg"] = list(data_file["parameters/prop_frequency/back_left/avg"])
            combined_params["prop_frequency"]["back_left"]["std_dev"] = list(data_file["parameters/prop_frequency/back_left/std_dev"])

        # Create a unique filename for the combined file
        combined_h5file = os.path.join(f"{data_folder + os.sep }combined", f"combined-stan-fpv-tilt-{params[0]}-{params[1]}m-fr-{params[2]}-fl-{params[3]}-br-{params[4]}-bl-{params[5]}.hdf5")
        os.makedirs(os.path.dirname(combined_h5file), exist_ok=True)
        with h5py.File(combined_h5file, 'w') as f:
            f.create_group("data")
            f["data/data"] = combined_params["data"]["data"]
            f["data/capture_time"] = combined_params["data"]["capture_time"]
            f["data/timestamps"] = combined_params["data"]["timestamps"]

            f.create_group("digitizer")
            f["digitizer/info"] = combined_params["digitizer"]["info"]

            f.create_group("digitizer/config")
            f["digitizer/config/acquisition"] = combined_params["digitizer"]["config"]["acquisition"]
            f["digitizer/config/trigger"] = combined_params["digitizer"]["config"]["trigger"]
            f["digitizer/config/channel"] = combined_params["digitizer"]["config"]["channel"]

            f.create_group("parameters")
            f["parameters/tilt"] = combined_params["tilt"]
            f["parameters/motor_configuration"] = combined_params["motor_configuration"]
            f["parameters/prop_size"] = combined_params["prop_size"]
            f["parameters/n_blades"] = combined_params["n_blades"]
            f["parameters/fill_factor"] = combined_params["fill_factor"]
            f["parameters/lens_tube_extension"] = combined_params["lens_tube_extension"]
            f["parameters/target_distance"] = combined_params["target_distance"]

            f.create_group("parameters/throttle")
            f["parameters/throttle/front_right"] = combined_params["throttle"]["front_right"]
            f["parameters/throttle/front_left"] = combined_params["throttle"]["front_left"]
            f["parameters/throttle/back_right"] = combined_params["throttle"]["back_right"]
            f["parameters/throttle/back_left"] = combined_params["throttle"]["back_left"]

            f.create_group("parameters/motor_rpm/front_right")
            f["parameters/motor_rpm/front_right/avg"] = combined_params["motor_rpm"]["front_right"]["avg"]
            f["parameters/motor_rpm/front_right/std_dev"] = combined_params["motor_rpm"]["front_right"]["std_dev"]

            f.create_group("parameters/motor_rpm/front_left")
            f["parameters/motor_rpm/front_left/avg"] = combined_params["motor_rpm"]["front_left"]["avg"]
            f["parameters/motor_rpm/front_left/std_dev"] = combined_params["motor_rpm"]["front_left"]["std_dev"]

            f.create_group("parameters/motor_rpm/back_right")
            f["parameters/motor_rpm/back_right/avg"] = combined_params["motor_rpm"]["back_right"]["avg"]
            f["parameters/motor_rpm/back_right/std_dev"] = combined_params["motor_rpm"]["back_right"]["std_dev"]

            f.create_group("parameters/motor_rpm/back_left")
            f["parameters/motor_rpm/back_left/avg"] = combined_params["motor_rpm"]["back_left"]["avg"]
            f["parameters/motor_rpm/back_left/std_dev"] = combined_params["motor_rpm"]["back_left"]["std_dev"]

            f.create_group("parameters/prop_frequency/front_right")
            f["parameters/prop_frequency/front_right/avg"] = combined_params["prop_frequency"]["front_right"]["avg"]
            f["parameters/prop_frequency/front_right/std_dev"] = combined_params["prop_frequency"]["front_right"]["std_dev"]

            f.create_group("parameters/prop_frequency/front_left")
            f["parameters/prop_frequency/front_left/avg"] = combined_params["prop_frequency"]["front_left"]["avg"]
            f["parameters/prop_frequency/front_left/std_dev"] = combined_params["prop_frequency"]["front_left"]["std_dev"]

            f.create_group("parameters/prop_frequency/back_right")
            f["parameters/prop_frequency/back_right/avg"] = combined_params["prop_frequency"]["back_right"]["avg"]
            f["parameters/prop_frequency/back_right/std_dev"] = combined_params["prop_frequency"]["back_right"]["std_dev"]

            f.create_group("parameters/prop_frequency/back_left")
            f["parameters/prop_frequency/back_left/avg"] = combined_params["prop_frequency"]["back_left"]["avg"]
            f["parameters/prop_frequency/back_left/std_dev"] = combined_params["prop_frequency"]["back_left"]["std_dev"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "data_folder",
        type=str,
        help="Path to the data folder",
    )

    args = parser.parse_args()

    sys.exit(
        combineH5(args.data_folder)
    )