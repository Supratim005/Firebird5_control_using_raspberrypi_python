# Firebird5_control_using_raspberrypi_python

line_tracking.py: MPC code for trajectory control

point_tracking.py: MPC code for point tracking.

motion.py: Main interfacing code for rpi with avr microcontroller of FB5.

actuator.py: Code for converting linear and angular velocity to corresponding PWM value for FB5 motor.

gps.py: Code for getting the position of the FB5 

used module:pyubx2
[
Installation:https://pypi.org/project/pyubx2/0.1.0/
github:https://github.com/semuconsulting/pyubx2
]

gps_test.py:Code for understanding ubx protocol

heading.py: Code for getting the heading of the FB5

vehicle.py:  I/P and O/P of the system

bno055.py: Code for testing bno005 sensor with rpi

Website: https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/software


