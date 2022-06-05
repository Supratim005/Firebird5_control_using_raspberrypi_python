import sys, select, os
import gps
import casadi as ca
import heading
import time
import board
import adafruit_bno055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)



def calibration():
	while True:
		os.system('cls' if os.name == 'nt' else 'clear')
		x,y,_ = gps.position()
		h = (360-sensor.euler[0])*(22/1260)
		if h==0 or h==360*(22/1260):
			h=0
		init_state = ca.vertcat(
		ca.horzcat(x), # East
		ca.horzcat(y), # North
		ca.horzcat(h)  # heading angle
		)
		print("I'm calibarating. Press Enter to stop!")
		print("X:",x,"Y:",y,"Heading:",h*(1260/22))
		time.sleep(1)
		if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
			return init_state 
			break


