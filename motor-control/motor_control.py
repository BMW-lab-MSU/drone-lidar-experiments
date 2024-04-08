import numpy as np
from time import sleep
from yamspy import MSPy
from yamspy.msp_codes import MSPCodes

# TODO: should this file be a class, or is having a module-level global for the MSPy object fine?
board = None

"""
Calling process routine:
>>> from multiprocessing import Process, Event, Pipe
>>> event = Event()
>>> parent_conn, child_conn = Pipe(False)
>>> rpm_collection_process = Process(target=motor_control.collect_rpm_data, args=(event, child_conn))
>>> # throw out bad telemetry before starting the process: motor_control.throw_out_old_telemetry()
>>> rpm_collection_process.start()
>>> event.set()
>>> event.clear()
>>> parent_conn.recv()
>>> rpm_collection_process.join()
"""

def get_rpm_telemetry():
    global board

    board.send_RAW_msg(MSPCodes['MSP_MOTOR_TELEMETRY'])
    recv = board.receive_msg()
    board.process_recv_data(recv)

    return np.array(board.MOTOR_TELEMETRY_DATA['rpm'][0:4])

def connect(serial_port):
    global board

    board = MSPy(serial_port)
    board.connect()

def set_throttle(throttle1, throttle2, throttle3, throttle4):
    global board

    board.send_RAW_MOTORS([throttle1, throttle2, throttle3, throttle4, 0, 0, 0, 0])

# NOTE: Using a pipe makes this only work with the multiprocessing module,
# but using a queue would make it work with threading and multiprocessing
# having to make any changes. Since we are sleeping in order to not overload
# the ESC or serial port, threading would work fine even though threading only
# executes one thread at a time.
def collect_rpm_data(event, pipe):
    # Set the sampling period so we don't overload the serial port or ESC.
    RPM_SAMPLING_PERIOD = 0.1 # seconds
    
    num_telemetry_packets = 0
    rpm = np.zeros((4,))

    event.wait()
    while event.is_set():
        rpm += get_rpm_telemetry()
        num_telemetry_packets += 1

        sleep(RPM_SAMPLING_PERIOD)
    
    # Average the rpm values
    avg_rpm = rpm / num_telemetry_packets

    pipe.send(avg_rpm)

def throw_out_old_telemetry():

    # For some reason, the rpm telemetry data takes a while to update, so the first handful
    # of rpm values we receive back after setting the rpm are not up-to-date.
    for i in range(0,10):
        get_rpm_telemetry()