'''

Created: 23-04-2015
Last Modified: 02-04-2022
Author: Saurav Shandilya (e-Yantra Team)
Modifier: Supratim Dey
Application: Development of Raspberry Pi controlled Firebird-V Robot.
Hardware: Firebird-V Atmega2560 Robot and Raspberry Pi 3
Prev Python Version: 2.7
New Python Version: 3.7
----------------------------------


'''

#***********************Library Import Starts*********************
import serial
import time
import glob			# Glob module finds all the pathnames matching a specified pattern. It is used for detecting serial ports in use
import sys			# This module provides access to some variables used or maintained by the interpreter. It is used to exit from program when exception occur

#***********************Library Import Ends*********************


#**********************Variable declaration Starts*********************
global device_id
global device_type
global function_type
global param_count
global port
global port_detect
barLED1 = 1
barLED2 = 2
barLED3 = 4
barLED4 = 8
barLED5 = 16
barLED6 = 32
barLED7 = 64
barLED8 = 128

#**********************Variable declaration Ends*********************

#**********************Communication/Serial Port Detection Starts*********************
def serial_port_connection(port_detect):
	global port
	
	print(len(port_detect),"Ports detected") # print number of ports detected
	
	#------------- print all detectec ports - STARTS ---------#
	if (len(port_detect) != 0):				
		print("Port(s) detected is/are:")	
		
		for i in range (0,len(port_detect)):
			print(port_detect[i])
	#------------- print all detectec ports - END ---------#
	
	#------------- connect to PORT if only one port is detected - STARTS ---------#
		if (len(port_detect) == 1):
			port = serial.Serial(port_detect[0],baudrate=9600)
			print("connected to: ", port_detect[0])
	#------------- connect to PORT if only one port is detected - END ---------#	
	
	#------------- Ask for user i/p if more then one port is detected - STARTS ---------#
		else:
			for i in range(0,len(port_detect)):
				print("Enter",i,"to connect to:",port_detect[i])
			
			y = int(raw_input("Enter your choice of connection: "))
			
			while y >= len(port_detect):
				print("Invalid choice")
				for i in range(0,len(port_detect)):
					print("Enter",i,"to connect to:",port_detect[i])
				y = int(raw_input("Enter your choice of connection: "))
	#------------- Ask for user i/p if more then one port is detected - END ---------#			
			
			port = serial.Serial(port_detect[y],baudrate=9600)
			print("connected to: ", port_detect[y])
	return
#**********************Communication/Serial Port Detection Ends*********************

#**********************Open Communication/Serial Port Starts*********************	
def serial_open():	
	port_detect = glob.glob("/dev/ttyUSB*") # stores all /dev/ttyUSB* into a list port_detect
	
	try:
		serial_port_connection(port_detect)
				
		if port.isOpen() == True:
			print("Port is open")
		else:
			serial_port_connection()
				
	except:
		print("No USB port detected....check connection")
		sys.exit(0)		# stop program execution when exception occur
#**********************Open Communication/Serial Port Starts*********************	


#**********************Close Communication/Serial Port Starts*********************	
def serial_close():
	port.close()
#**********************Close Communication/Serial Port Ends*********************


#**********************Forward Starts*********************	
def forward ():
	 print("Forward Motion")
	 data = []
	 device_id = 2				#DC Motors has device id = 2
	 device_type = 1			#DC Motors is o/p device. hence device type = 0	
	 function_type = 0			#Function_type = 0 for forward motion
	 param_count = 0			#No parameter is sent through forward function hence param_count = 0
	 data.append(chr(device_id))
	 data.append(chr(device_type))
	 data.append(chr(function_type))
	 data.append(chr(param_count))
	 data.append("\n")
	 data.append("\r")
	
	 for i in range(0,len(data)):
		 port.write((str(data[i])).encode())
		 print(str(data[i]))
	 print("packet sent is" , str(data))
	 return
#**********************Forward Ends*********************

#**********************Back Starts*********************
def back ():
	 print("Back Motion")
	 data = []
	 device_id = 2				#DC Motors has device id = 2
	 device_type = 1			#DC Motors is o/p device. hence device type = 0	
	 function_type = 1			#Function_type = 0 for forward motion
	 param_count = 0			#No parameter is sent through forward function hence param_count = 0
	 data.append(chr(device_id))
	 data.append(chr(device_type))
	 data.append(chr(function_type))
	 data.append(chr(param_count))
	 data.append("\n")
	 data.append("\r")
	
	 for i in range(0,len(data)):
		 port.write((str(data[i])).encode())
		 print(str(data[i]))
	
	 print("packet sent is" , str(data))
	 return
#**********************Back Starts*********************

#**********************Stop Starts*********************	
def stop ():
	 print("Stopping DC Motors")
	 device_id = 2
	 device_type = 1
	 function_type = 4
	 param_count = 0
	 port.write((chr(device_id)).encode())
	 port.write((chr(device_type)).encode())
	 port.write((chr(function_type)).encode())
	 port.write((chr(param_count)).encode())
	 port.write(("\n").encode())
	 port.write(("\r").encode())
	 return
#**********************Stop Ends*********************





#**********************Velocity Control Starts*********************
def velocity(left_motor,right_motor,mode):
	print("Left motor velocity = ", '%s' %str(left_motor), "Right motor velocity = ",'%s' %str(right_motor))
	data = []
	device_id = 2
	device_type = 1
	function_type = 9
	param_count = 3
	param_1 = left_motor
	param_2 = right_motor
	param_3 = mode
	data.append(chr(device_id))
	data.append(chr(device_type))
	data.append(chr(function_type))
	data.append(chr(param_count))
	data.append((param_1))
	data.append((param_2))
	data.append((param_3))
	data.append("\n")
	data.append("\r")
	
	for i in range(0,len(data)):
		if i==4:
				port.write(bytes([data[i]]))
				print(bytes([data[i]]))
		elif i==5:
				port.write((bytes([(data[i])])))
				print(bytes([data[i]]))
		elif i==6:
				port.write((bytes([(data[i])])))
				print(bytes([data[i]]))
		else:
			port.write((str((data[i]))).encode())
	
	print("packet sent is" , (str(data)))
	return
#**********************Velocity Control Ends*********************
