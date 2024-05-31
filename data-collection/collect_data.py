import sys
import multiprocessing
import tomllib
import argparse
import pandas as pd
import time

import motor_control
from quickset_pan_tilt import controller, protocol
import wingbeat_lidar as lidar

# TODO: take in range calibration file. make it optional, though.

# Initialize digitizer
def setup_digitizer(config_file="./config/adc-config.toml"):
    digitizer = lidar.digitizer.Digitizer(config_file)
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
    # Setup multiprocessing
    collect_rpm = multiprocessing.Event()
    experiment_active = multiprocessing.Event()
    rpm_recv_pipe, rpm_send_pipe = multiprocessing.Pipe(False)
    rpm_collection_process = multiprocessing.Process(
        target=motor_control.collect_rpm_data, args=(stop_rpm_collection, rpm_send_pipe)
    )

    motor_control.connect(port)

    return collect_rpm, experiment_active, rpm_recv_pipe, rpm_collection_process


def load_port_configuration(config_file="./config/serial-ports.toml"):
    with open(config_file, "rb") as f:
        config = tomllib.load()

    return config.pan_tilt_port, config.drone_port


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


def set_tilt_angle(pan_tilt, experiment_params, idx):
    # XXX: This function probably isn't necessary. We could just move every time,
    # even if the tilt angle hasn't changed. The pan tilt mount will just ack
    # the command and then not move.
    if idx > 0:
        current_params = experiment_params.iloc[idx]
        past_params = experiment_params.iloc[idx - 1]

        tilt_angle_changed = current_params["tilt angle"] != past_params["tilt angle"]
        if tilt_angle_changed:
            pan_tilt.move_absolute(0, current_params["tilt angle"])
    else:
        pan_tilt.move_absolute(0, params["tilt angle"])


def set_throttle(experiment_params, idx):
    # Make the motors ramp to the new rpm value over 5 seconds. This seems like
    # a safe value that won't cause any super crazy current spikes, but the
    # value is ultimately arbitrary.
    RAMP_TIME = 5

    front_left_throttle = experiment_params.iloc[idx]["throttle front left"]
    front_right_throttle = experiment_params.iloc[idx]["throttle front right"]
    back_left_throttle = experiment_params.iloc[idx]["throttle back left"]
    back_right_throttle = experiment_params.iloc[idx]["throttle back right"]

    motor_control.set_throttle(
        [
            back_left_throttle,
            front_left_throttle,
            back_right_throttle,
            front_right_throttle,
        ],
        ramp_time=RAMP_TIME,
    )


def save_rpm_in_dataframe(experiment_params, idx, avg_rpm, std_dev_rpm):
    """
    
    """
    experiment_params.iloc[idx]["motor rpm front right"] = avg_rpm[:, 3]
    experiment_params.iloc[idx]["motor rpm front left"] = avg_rpm[:, 1]
    experiment_params.iloc[idx]["motor rpm back right"] = avg_rpm[:, 2]
    experiment_params.iloc[idx]["motor rpm back left"] = avg_rpm[:, 0]

    experiment_params.iloc[idx]["motor rpm front right std dev"] = std_dev_rpm[:, 3]
    experiment_params.iloc[idx]["motor rpm front left std dev"] = std_dev_rpm[:, 1]
    experiment_params.iloc[idx]["motor rpm back right std dev"] = std_dev_rpm[:, 2]
    experiment_params.iloc[idx]["motor rpm back left std dev"] = std_dev_rpm[:, 0]

    experiment_params.iloc[idx]["prop frequency front right"] = _compute_prop_frequency(
        avg_rpm[:, 3]
    )
    experiment_params.iloc[idx]["prop frequency front left"] = _compute_prop_frequency(
        avg_rpm[:, 1]
    )
    experiment_params.iloc[idx]["prop frequency back right"] = _compute_prop_frequency(
        avg_rpm[:, 2]
    )
    experiment_params.iloc[idx]["prop frequency back left"] = _compute_prop_frequency(
        avg_rpm[:, 0]
    )

    experiment_params.iloc[idx]["prop frequency front right std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 3])
    )
    experiment_params.iloc[idx]["prop frequency front left std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 1])
    )
    experiment_params.iloc[idx]["prop frequency back right std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 2])
    )
    experiment_params.iloc[idx]["prop frequency back left std dev"] = (
        _compute_prop_frequency(std_dev_rpm[:, 0])
    )


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
        motor_rpm:

        experiment_params:

        idx:
    """
    n_blades = experiment_params.iloc[idx]["# blades"]

    # Conversion for revolutions per minute to revolutions per second
    RPM_TO_HZ = 1 / 60

    prop_frequency = motor_rpm * n_blades * RPM_TO_HZ

    return prop_frequency


def create_h5_filename(experiment_params, idx):
    tilt_angle = f"tilt-{experiment_params.iloc[idx]['tilt angle']}"

    throttle_fr = ""
    throttle_fl = ""
    throttle_br = ""
    throttle_bl = ""
    if isinstance(throttle_front_right, int):
        throttle_fr = f"-fr-{experiment_params.iloc['throttle front right']}"
    if isinstance(throttle_front_left, int):
        throttle_fl = f"-fl-{experiment_params.iloc['throttle front left']}"
    if isinstance(throttle_back_right, int):
        throttle_br = f"-br-{experiment_params.iloc['throttle back right']}"
    if isinstance(throttle_back_left, int):
        throttle_bl = f"-bl-{experiment_params.iloc['throttle back left']}"

    timestamp = f"-{time.strftime('%H-%M-%S')}"

    filename = (
        tilt_angle + throttle_fr + throttle_fl + throttle_br + throttle_bl + timestamp
    )

    return filename


def save_h5_file(
    h5_filename, data, timestamps, capture_time, avg_rpm, std_dev_rpm, digitizer, is_data_in_volts, distance
):
    pass


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

    experiment_params = pd.read_excel(experiment_spreadsheet_path)

    pan_tilt_port, drone_port = load_port_configuration()

    pan_tilt = setup_pan_tilt_controller(pan_tilt_port)

    collect_rpm, experiment_active, rpm_recv_pipe, rpm_collection_process = (
        setup_drone_controller(drone_port)
    )

    digitizer = setup_digitizer()

    # Read in ground-truth / experiment parameter spreadsheet so we have the
    # experiment parameters we need, i.e., motor speed, tilt angle, etc.

    # TODO: prompt for how far away the drone is? probably not; we can just save manually enter that in the spreadsheet.

    # TODO: prompt for distance between lenses when the fill factor changes

    # Get the image size that the digitizer is collecting so we can preallocate
    # matrices later for saving the data.
    n_samples = digitizer.acquisition_config.SegmentSize
    n_segments = digitizer.acquisition_config.SegmentCount

    # Spawn and start the process that collects rpm data from the drone.
    collect_rpm.start()

    # Tell the rpm collection process that it can run its main loop. The
    # process will won't collect any rpm data until collect_rpm is set.
    # However, we need this extra flag to tell the process to terminate
    # so we can join the process once everything is done; we need this flag
    # because we can't call the process's start method more than once; while
    # we could make a new process for every image, that would not be efficient.
    experiment_active.set()

    for idx, params in experiment_params.iterrows():
        if is_manual_adjusted_needed(experiment_params, idx):
            answer = "n"
            while answer.lower() != y:
                answer = input(
                    'Press "y" when you are ready to run the next configuration'
                )

        set_tilt_angle(pan_tilt, experiment_params, idx)

        set_throttle(experiment_params, idx)

        n_images = experiment_params.iloc[idx]["# images"]

        data = np.empty((n_images, n_samples, n_segments))
        timestamps = np.empty(shape=(n_images, n_segments))
        capture_time = np.empty(shape=n_images, dtype=np.bytes_)

        avg_rpm = np.empty((n_images, N_MOTORS))
        std_dev_rpm = np.empty((n_images, N_MOTORS))

        for image_num in range(n_images):

            # Tell the process to start collecting rpm telemetry
            collect_rpm.set()

            # Collect the lidar data
            (
                data[image_num, :, :],
                timestamps[image_num, :],
                capture_time[image_num],
            ) = digitizer.capture()

            # We're done collecting data, so stop collecting rpm telemetry
            collect_rpm.clear()

            # Get the average rpm values
            (avg_rpm[image_num, :], std_dev_rpm[image_num, :]) = rpm_recv_pipe.recv()

        # Put the ground-truth rpm data into the dataframe
        save_rpm_in_dataframe(experiment_params, idx, avg_rpm, std_dev_rpm)

        if use_volts:
            data = digitizer.convert_to_volts(data)

        # Save the data, ground-truth, and metadata in an h5 file
        h5_filename = create_h5_filename(experiment_params, idx)

        save_h5_file(
            h5_filename, data, timestamps, capture_time, avg_rpm, std_dev_rpm, digitizer, use_volts, distance
        )

        # Put the data filename in the ground-truth dataframe
        experiment_params.iloc[idx]["filename"] = h5_filename

        # Save the spreadsheet just in case something in the experiment blows
        # up causing us to kill this code midway through the experiments.
        experiment_params.to_excel(experiment_spreadsheet_path)

    # The experiment is over; tell the rpm collection process that it can
    # terminate itself.
    experiment_active.clear()

    # Stop the rpm collection process
    rpm_collection.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "experiment-spreadsheet",
        type=str,
        help="Path to the experiment parameters spreadsheet",
    )
    parser.add_argument(
        "data-directory", type=str, help="Path to the top-level data directory"
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
