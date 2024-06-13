# Data collection config files

- `99-usb-serial-drone-controller.rules`: udev rule to make the drone's serial port show up as /dev/drone. Copy this to /etc/udev/rules.d/
- `serial-ports.toml`: Drone and pan-tilt mount serial port names
- `digitizer.toml`: Digitizer configuration
- `range-calibration.toml`: Range calibration configuration, as created by running the wingbeat_lidar range_calibration module
