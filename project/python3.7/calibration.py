import sys, select, os
import gps
import casadi as ca
import heading
import time


def calibration():
	while True:
		os.system('cls' if os.name == 'nt' else 'clear')
		x,y,_ = gps.position()
		h = heading.calibration()
		init_state = ca.vertcat(
		ca.horzcat(x), # East
		ca.horzcat(y), # North
		ca.horzcat(h)  # heading angle
		)
		print("I'm calibarating. Press Enter to stop!")
		print("X:",x,"Y:",y,"Heading:",h)
		time.sleep(1)
		if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
			return init_state 
			break
