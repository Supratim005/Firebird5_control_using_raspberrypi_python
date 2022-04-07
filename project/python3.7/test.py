import motion as pi 
import time

pi.serial_open()
x=95
while True:
		#pi.forward()
		#time.sleep(2)
		#pi.back()
		#time.sleep(2)
		#pi.stop()
		#time.sleep(2)
		
		#pi.velocity(0,255)# it's working
		
		#pi.velocity(110,110,1)# not working
		#time.sleep(5)
		pi.velocity(0,x,2)
		time.sleep(5)
		#pi.velocity(0,0)
		#time.sleep(3)
		#pi.velocity(255,255)
		#time.sleep(3)
		#pi.stop()
		#time.sleep(3)
		x=x+1;
pi.serial_close()				# close serial portCC