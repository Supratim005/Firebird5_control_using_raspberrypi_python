'''
Created: 23-04-2015 
Last Modified: 23-06-2015
Author: Saurav Shandilya (e-Yantra Team)
Application: Development of Raspberry Pi controlled Firebird-V Robot.
Hardware: Firebird-V Atmega2560 Robot and Raspberry Pi 1
Python Version: 2.7

Description:
1. This is a experiment file which will test all hardware and their components.
2. Current Implementation includes hardware test like Buzzer, DC motors with velocity control, Servo motors, Interrupt driven position encoder, BarLED, Sensors reading from both ATmega2560(master) and ATmega 8(slave) microcontroller.

--------------------------------------------------------------------
Usage:
Run the program
Prompt will ask to enter a number:

0 - Stop DC motor, turn off buzzer  and barLED, 
1 - DC motor direction control and Velocity control
2 - Travel bot in forward/backward direction by given distance (note distance is in millimeters)
3 - Servo motor control
4 - BarLED control
5 - ADC sensors value 

'''

import fipi as pi 
import time

pi.serial_open()				# Open serial port

while True:
	
	y= raw_input("Enter the number:")
	
	# 0 - Stop Motion and buzzer
	if y == '0':
		pi.stop()
		pi.buzzer_off
		print "good bye"
		break
		
	# 1 - Motion Control Test	
	if y == '1':
        
		#pi.forward()
		#time.sleep(5)
		#pi.back()
		#time.sleep(5)
		#pi.stop()
		#time.sleep(2)
		
		#pi.velocity(255,255)
		#time.sleep(2)
		pi.velocity(255,0)
		#time.sleep(2)
		#pi.velocity(0,0)
		time.sleep(3)
		pi.velocity(0,255)
		time.sleep(3)
		
	# 2 - Distance travelled Test	
	if y == '2':
		pi.forward_mm(1000) 	# distance in mm. maximum allowable value for distanceinmm = 65535
		time.sleep(2)
		pi.back_mm(300)
		time.sleep(1)
	
	# 3 - Servo motor control Test	
	if y == '3':
		pi.servo_1(0)
		pi.servo_2(0)
		time.sleep(0.5)			# delay is mandatory to ensure motion is complete before next rotation degree is sent
	
		pi.servo_1(120)
		pi.servo_2(120)
	
		time.sleep(0.5)

		pi.servo_1_free()
		pi.servo_2_free()

	#4 - BarLED
	if y == '4':
		pi.led_bargraph_on(pi.barLED1)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4|pi.barLED5)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4|pi.barLED5|pi.barLED6)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4|pi.barLED5|pi.barLED6|pi.barLED7)
		time.sleep(1)
		pi.led_bargraph_on(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4|pi.barLED5|pi.barLED6|pi.barLED7|pi.barLED8)
		time.sleep(2)
		
		pi.led_bargraph_off(pi.barLED1|pi.barLED2|pi.barLED3|pi.barLED4|pi.barLED5|pi.barLED6|pi.barLED7|pi.barLED8)
	
	# 5 - ADC sensors values	
	if y == '5':
		pi.adc_conversion(1)			# Channel No-1 of ATmega2560
		#time.sleep(1)
		pi.adc_conversion(2)			# Channel No-2 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(3)			# Channel No-3 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(4)			# Channel No-4 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(5)			# Channel No-5 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(6)			# Channel No-6 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(7)			# Channel No-7 of ATmega2560
		time.sleep(0.1)
		pi.adc_conversion(8)			# Channel No-8 of ATmega2560
		time.sleep(0.1)
		pi.spi_master_tx_and_rx(1)		# Channel No-1 of ATmega8
		pi.spi_master_tx_and_rx(5)		# Channel No-5 of ATmega8
		time.sleep(0.1)
		pi.spi_master_tx_and_rx(6)		# Channel No-6 of ATmega8	
		pi.spi_master_tx_and_rx(7)		# Channel No-7 of ATmega8
		time.sleep(0.1)
		
pi.serial_close()				# close serial port
