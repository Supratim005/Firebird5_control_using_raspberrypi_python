import fipi as pi 
import time

pi.serial_open()
x=0
while x<1:
		#pi.forward()
		#time.sleep(5)
		#pi.back()
		#time.sleep(5)
		#pi.stop()
		#time.sleep(2)
		
		pi.velocity(255,0)
		#time.sleep(2)
		#pi.velocity(255,0)
		#time.sleep(2)
		#pi.velocity(0,0)
		#time.sleep(3)
		#pi.velocity(255,255)
		#time.sleep(3)
		#pi.stop()
		#time.sleep(3)
		x=x+1
pi.serial_close()				# close serial port