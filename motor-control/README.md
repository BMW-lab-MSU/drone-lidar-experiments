# Motor control helper
This package is a wrapper to make controlling and monitoring motor RPM easier.

For drones using firmware that supports the MultiWii Serial Protocol (MSP)
(e.g, BetaFlight, iNav, etc.), this module allows users to
1. control the throttle of drone motors.
2. receive RPM telemetry from the ESC and get an average rpm reading over a period time.

This module wraps the [YAMSPy library](https://github.com/thecognifly/YAMSPy);
currently, only [our fork](https://github.com/BMW-lab-MSU/YAMSPy) has support
for receiving motor telemetry, so that is what gets installed as a dependency.

## Typical usage
The intent is that the rpm collection will run in the background while the digitizer is collecting data, thus the module is designed to use the multiprocessing library.

Here's a typical usage example showing the flow of the main program:
```python
from multiprocessing import Process, Event, Pipe

event = Event()
parent_conn, child_conn = Pipe(False)
rpm_collection_process = Process(target=motor_control.collect_rpm_data, args=(event, child_conn))

motor_control.connect("/dev/ttyACM0")
motor_control.set_throttle(1500,1500,1500,1500)
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
```

## Installation

Install the package using `pip install .` in this directory.
