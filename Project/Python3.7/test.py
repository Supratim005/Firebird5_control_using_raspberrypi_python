import motion as pi 
import time
import actuator
import csv
import board
import adafruit_bno055
import board
import keyboard
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

pi.serial_open()
x=1

'''
with open('pwm.csv','r') as csv_file:
	csv_reader=csv.reader(csv_file)
	
	for i in csv_reader:
		x=x+1
		print("instance:",x)
		#actuator.control_ip(float(i[0]),float(i[1]),.05,0.18)
		#actuator.control_ip(0.05,0.7,.05,0.18)
		pi.velocity(int(i[0]),int(i[1]),1)
		#time.sleep(1)
		#pi.stop()
		time.sleep(1)
		pi.stop()
		time.sleep(2)
		#print("yaw angle: {}".format(sensor.euler[0]))
		#time.sleep(poll_interval*1.0/1000.0)
		#time.sleep(1)

'''
while True:
		#pi.forward()
		#time.sleep(2)
		#pi.back()
		#time.sleep(2)
		#pi.stop()
		#time.sleep(2)
		
		#pi.velocity(0,255)# it's working
		
		#pi.velocity(255,255,2)# not working
		#time.sleep(5)
		#pi.velocity(0,x,2)
		#actuator.control_ip(0.0152,0.1815,.05,0.18)
		#time.sleep(1)
		#pi.velocity(0,255,1)
		#time.sleep(3)
		pi.velocity(247,0,1)
		#actuator.control_ip(0.0352400516511952,0.329161446487067,0.05,0.18)
		time.sleep(1)
		
		pi.stop()
		#pi.stop()
		time.sleep(1)
		print("yaw angle: {}".format(sensor.euler[0]))
		#x=x+1	
		break	
	
pi.serial_close()			# close serial portCC

