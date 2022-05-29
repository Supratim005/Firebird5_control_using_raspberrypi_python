import calibration
import heading
import time
state_init = calibration.calibration()        # initial state

while True:
	heading.heading()
	time.sleep(1)
