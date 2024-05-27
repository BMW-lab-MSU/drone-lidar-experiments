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


def get_rpm_telemetry():
    """Get RPM telemetry values from the ESC.

    Returns:
        rpm: A numpy array of the 4 motor RPM values.
    """
    global board

    board.send_RAW_msg(MSPCodes["MSP_MOTOR_TELEMETRY"])
    recv = board.receive_msg()
    board.process_recv_data(recv)

    return np.array(board.MOTOR_TELEMETRY_DATA["rpm"][0:4])


def connect(serial_port):
    """Connect to the drone's FC.

    Args:
        serial_port: ACM serial port that the drone is connected to.
    """
    global board

    board = MSPy(serial_port)
    board.connect()


def set_throttle(throttle, ramp_time=None, ramp_interval=0.1):
    """Set motor throttle.

    Motor throttle values for MSP are between 1000 (0%) and 2000 (100%).

    Args:
        throttle:
            An iterable of the throttle values for the motors. Must have a
            length of 4.
        ramp_time:
            Ramp motors to their final throttle values over this time. Defaults
            to None, which means no ramp time.
        ramp_interval:
            The interval, in seconds, at which the motor throttle values are
            updated during the ramp time. Defaults to 0.1 seconds.
    """
    global board

    ZERO_PCT_THROTTLE = 1000

    final_throttle = throttle

    if ramp_time:
        n_steps = round(ramp_time / ramp_interval)

        intermediate_throttle = np.ndarray(shape=(4, n_steps))

        intermediate_throttle[0, :] = np.linspace(
            ZERO_PCT_THROTTLE, final_throttle[0], n_steps, dtype=np.uint
        )
        intermediate_throttle[1, :] = np.linspace(
            ZERO_PCT_THROTTLE, final_throttle[1], n_steps, dtype=np.uint
        )
        intermediate_throttle[2, :] = np.linspace(
            ZERO_PCT_THROTTLE, final_throttle[2], n_steps, dtype=np.uint
        )
        intermediate_throttle[3, :] = np.linspace(
            ZERO_PCT_THROTTLE, final_throttle[3], n_steps, dtype=np.uint
        )

        for i in range(n_steps):
            print(intermediate_throttle[:, i])
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


def collect_rpm_data(event, pipe):
    """Collect RPM telemetry data in the background.

    Collects RPM telemetry from the FC/ESC every ~0.1 seconds. This method is
    designed to run in the background as a separate process while the main
    process is doing something else. At this time, only the multiprocessing
    module is supported.
    
    The average and standard deviation of the RPM values are sent over the
    pipe as a tuple: (avg_rpm, rpm_std_dev). The caller must receive these
    values on its pipe.

    Args:
        event:
            multiprocessing.Event used to control when rpm collection starts.
        pipe:
            multiprocessing.Pipe used to return the average RPM to the caller.
    """
    # Set the sampling period so we don't overload the serial port or ESC.
    # NOTE: YAMSPy's maximum serial send rate is 1/100 seconds
    RPM_SAMPLING_PERIOD = 0.01  # seconds

    # This buffer length lets us collect up to 1000 seconds of rpm data, which
    # is 16.66 minutes. We don't envision a scenario where we collect more than
    # a handful of seconds of rpm data at a time, but we might as well
    # over-allocate the array to be safe.
    RPM_BUFFER_LENGTH = 100_000

    N_MOTORS = 4

    rpm = np.empty((N_MOTORS, RPM_BUFFER_LENGTH))

    n_telemetry_packets = 0
    packet_idx = 0

    event.wait()
    while event.is_set():
        rpm[:,packet_idx] = get_rpm_telemetry()
        packet_idx += 1
        n_telemetry_packets += 1

        sleep(RPM_SAMPLING_PERIOD)

    avg_rpm = np.mean(rpm[:,0:n_telemetry_packets], axis=1)
    rpm_std_dev = np.std(rpm[:,0:n_telemetry_packets], axis=1)

    # NOTE: Using a pipe makes this only work with the multiprocessing module,
    # but using a queue would make it work with threading and multiprocessing
    # having to make any changes. Since we are sleeping in order to not overload
    # the ESC or serial port, threading would work fine even though threading only
    # executes one thread at a time.
    pipe.send((avg_rpm, rpm_std_dev))


def throw_out_old_telemetry():
    """Read outdated telemetry values.

    For some reason, the rpm telemetry data takes a while to update, so the
    first handful of rpm values we receive back after setting the rpm are not
    up-to-date. This function reads 10 telemetry packets, which, in practice,
    has been enough to flush the outdated telemetry packets.
    """

    for i in range(0, 10):
        get_rpm_telemetry()
