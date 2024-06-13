"""Wrappers to make controlling and monitoring motor RPM easier.

For drones using firmware that supports the MultiWii Serial Protocol (MSP)
(e.g, BetaFlight, iNav, etc.), this module allows users to
1. control the throttle of drone motors.
2. receive RPM telemetry from the ESC and get an average rpm reading over a period time.

This module wraps the `YAMSPy library <https://github.com/thecognifly/YAMSPy>`_;
currently, only `our fork <https://github.com/BMW-lab-MSU/YAMSPy>`_ has support
for receiving motor telemetry.

Typical usage example:
    from multiprocessing import Process, Event, Pipe

    event = Event()
    parent_conn, child_conn = Pipe(False)
    rpm_collection_process = Process(target=motor_control.collect_rpm_data, args=(event, child_conn))

    motor_control.connect("/dev/ttyACM0")
    motor_control.set_throttle([1500,1500,1500,1500])
    motor_control.throw_out_old_telemetry()

    rpm_collection_process.start()

    # signal the rpm collection process to start
    event.set()

    # do other stuff
    ...

    # signal the rpm collection process to stop
    event.clear()

    avg_rpm = parent_conn.recv()

    rpm_collection_process.join()
"""

# SPDX-License-Identifier: GPL-3.0-only

import numpy as np
from time import sleep
from yamspy import MSPy
from yamspy.msp_codes import MSPCodes

# This module could have been a class, but using a module-level global seemed
# fine since the only internal state we need is the MSPy board object.
board = None
current_throttle = [0, 0, 0, 0]


def get_rpm_telemetry():
    """Get RPM telemetry values from the ESC.

    Returns:
        rpm: A numpy array of the 4 motor RPM values.
    """
    global board

    board.send_RAW_msg(MSPCodes["MSP_MOTOR_TELEMETRY"])
    recv = board.receive_msg(delete_buffer=True)

    if recv['dataView']:
        board.process_recv_data(recv)
        return np.array(board.MOTOR_TELEMETRY_DATA["rpm"][0:4])
    else:
        return None


def connect(serial_port):
    """Connect to the drone's FC.

    Args:
        serial_port: ACM serial port that the drone is connected to.
    """
    global board

    board = MSPy(serial_port, loglevel='DEBUG', timeout=0.1, trials=5)
    board.connect()


def set_throttle(throttle, ramp_interval=0.1):
    """Set motor throttle.

    Motor throttle values are between 0 and 100, with a resolution of 0.1.

    Args:
        throttle:
            An iterable of the throttle values for the motors. Must have a
            length of 4. When looking at the front of the drone, the motor
            order is [back left, front left, back right, front right].
        ramp_interval:
            The interval, in seconds, at which the motor throttle values are
            updated during the ramp time. Defaults to 0.1 seconds.
    """
    global board
    global current_throttle

    starting_throttle = _convert_pct_throttle_to_msp(current_throttle)

    final_throttle = _convert_pct_throttle_to_msp(throttle)

    ramp_time = np.max(np.abs(np.array(throttle) - np.array(current_throttle))) / 20

    if ramp_time != 0:
        n_steps = round(ramp_time / ramp_interval)

        intermediate_throttle = np.ndarray(shape=(4, n_steps))

        intermediate_throttle[0, :] = np.linspace(
            starting_throttle[0], final_throttle[0], n_steps, dtype=np.uint
        )
        intermediate_throttle[1, :] = np.linspace(
            starting_throttle[1], final_throttle[1], n_steps, dtype=np.uint
        )
        intermediate_throttle[2, :] = np.linspace(
            starting_throttle[2], final_throttle[2], n_steps, dtype=np.uint
        )
        intermediate_throttle[3, :] = np.linspace(
            starting_throttle[3], final_throttle[3], n_steps, dtype=np.uint
        )

        for i in range(n_steps):
            # print(intermediate_throttle[:, i])
            board.send_RAW_MOTORS(
                [
                    intermediate_throttle[0, i],
                    intermediate_throttle[1, i],
                    intermediate_throttle[2, i],
                    intermediate_throttle[3, i],
                    0,
                    0,
                    0,
                    0,
                ]
            )

            sleep(ramp_interval)

    else:
        board.send_RAW_MOTORS(
            [
                final_throttle[0],
                final_throttle[1],
                final_throttle[2],
                final_throttle[3],
                0,
                0,
                0,
                0,
            ]
        )

    current_throttle = throttle


def collect_rpm_data(collect_rpm, run_main_loop, telemetry_stable, pipe):
    """Collect RPM telemetry data in the background.

    Collects RPM telemetry from the FC/ESC every ~0.1 seconds. This method is
    designed to run in the background as a separate process while the main
    process is doing something else. At this time, only the multiprocessing
    module is supported.

    The average and standard deviation of the RPM values are sent over the
    pipe as a tuple: (avg_rpm, rpm_std_dev). The caller must receive these
    values on its pipe.

    Args:
        collect_rpm:
            multiprocessing.Event used to control when rpm telemetry is collected.
        run_main_loop:
            multiprocessing.Event used to signal when the process should run.
        pipe:
            multiprocessing.Pipe used to return the average RPM to the caller.
    """

    # This buffer length lets us collect up to 1000 seconds of rpm data, which
    # is 16.66 minutes. We don't envision a scenario where we collect more than
    # a handful of seconds of rpm data at a time, but we might as well
    # over-allocate the array to be safe.
    RPM_BUFFER_LENGTH = 100_000

    N_MOTORS = 4


    run_main_loop.wait()
    while run_main_loop.is_set():

        rpm = np.zeros((N_MOTORS, RPM_BUFFER_LENGTH))

        n_telemetry_packets = 0
        packet_idx = 0

        collect_rpm.wait()

        if not telemetry_stable.is_set():
            throw_out_old_telemetry()

            telemetry_stable.set()

        while collect_rpm.is_set():
            
            current_packet = get_rpm_telemetry()

            if current_packet is not None:
                rpm[:,packet_idx] = current_packet
                packet_idx += 1
                n_telemetry_packets += 1

        avg_rpm = np.mean(rpm[:,0:n_telemetry_packets], axis=1)
        rpm_std_dev = np.std(rpm[:,0:n_telemetry_packets], axis=1)

        # NOTE: Using a pipe makes this only work with the multiprocessing module,
        # but using a queue would make it work with threading and multiprocessing
        # having to make any changes. Since we are sleeping in order to not overload
        # the ESC or serial port, threading would work fine even though threading only
        # executes one thread at a time.
        pipe.send((avg_rpm, rpm_std_dev))


def arm():
    """Arm the flight controller.

    Manually arming the flight controller enables unlimited bench testing,
    which is needed in order to manually control the motors without
    going into a failsafe or runaway takeoff prevention mode.
    """
    global board
    
    # When enabling motor testing mode, betaflight configurator disables
    # armingDisabled and enables runawayTakeoffPreventionDisabled; that is,
    # arming is enabled and runaway takeoff prevention is disabled.
    # https://github.com/betaflight/betaflight-configurator/blob/2f4337bb9fabb6c9c56cf8e4b9144bda134910fb/src/js/tabs/motors.js#L945
    # https://github.com/betaflight/betaflight-configurator/blob/2f4337bb9fabb6c9c56cf8e4b9144bda134910fb/src/js/msp/MSPHelper.js#L2758
    board.set_ARMING_DISABLED(armingDisabled=0, runawayTakeoffPreventionDisabled=1)


def throw_out_old_telemetry():
    """Read outdated telemetry values.

    For some reason, the rpm telemetry data takes a while to update, so the
    first handful of rpm values we receive back after setting the rpm are not
    up-to-date. This function reads 10 telemetry packets, which, in practice,
    has been enough to flush the outdated telemetry packets.
    """

    # Throw out old buffered data
    rpm = get_rpm_telemetry()
    while rpm is not None:
        rpm = get_rpm_telemetry()

    # Once the rpm telemetry is reporting None, that means the buffer has been
    # flushed, so we can start collecting the updated rpm telemetry.
    while rpm is None:
        rpm = get_rpm_telemetry()

    # Throw out some more rpm data, just in case.
    for i in range(100):
        rpm = get_rpm_telemetry()


def _convert_pct_throttle_to_msp(throttle_pct):
    """Convert from percent throttle to MSP throttle values.

    The MultiWii Serial Protocol expects throttle values that range from
    1000 to 2000, with 1000 being 0% throttle and 2000 being 100%. Using
    percentages is easier to understand, thus the motor control API uses them.

    Args:
        throttle_pct:
            Iterable of throttle percentages to be converted. Maximum
            resolution is 0.1, e.g., 50.3.

    Returns:
        throttle_msp:
            List of throttle values in MSP format.

    """
    MULTIPLIER = 10
    OFFSET = 1000

    # MSP throttle values must be integers.
    throttle_msp = [int(pct * MULTIPLIER + OFFSET) for pct in throttle_pct]

    return throttle_msp
