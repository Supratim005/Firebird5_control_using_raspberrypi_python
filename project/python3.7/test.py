import motion as pi 
import time

pi.serial_open()
x=36
while True:
		#pi.forward()
		#time.sleep(5)
		#pi.back()
		#time.sleep(5)
		#pi.stop()
		#time.sleep(2)
		
		#pi.velocity(0,255)# it's working
		
		pi.velocity(255,0,2)# not working
		#time.sleep(2)
		#pi.velocity(0,255,1)
		time.sleep(2)
		#pi.velocity(0,0)
		#time.sleep(3)
		#pi.velocity(255,255)
		#time.sleep(3)
		#pi.stop()
		#time.sleep(3)
		x=x+1;
pi.serial_close()				# close serial portCC