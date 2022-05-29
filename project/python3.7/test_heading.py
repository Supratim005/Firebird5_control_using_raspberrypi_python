import calibration
import heading

import time

state_init = calibration.calibration()        # initial state
#print(state_init[2])
import board
import adafruit_bno055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

x=1
while True:
	y=heading.heading(sensor)
	y=state_init[2]-y
	print(y*(1260/22))
	time.sleep(1)
