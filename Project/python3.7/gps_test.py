import serial
import io
import time
import math
import numpy as np
ser=serial.Serial('/dev/ttyACM0',19200)


def binarytodecimal(k,l):
	decimal = 0
	binary = []
	for i in range(4):
		nzeros = 10 - len((k[l-i]))
		print(nzeros)
		for j in range(nzeros):
			binary.append(0)
		for j in range(len(k[l-i])-2):
				binary.append(k[l-i][j+2])      #Not appending initial 0b

	if(int(k[l][2])==1):
		for i in range(len(binary)):
			if(binary[i] == 1):
				binary[i] = 0
			else:
				binary[i] = 1
			decimal = decimal + int(binary[i])*pow(2,(len(binary)-(i+1)))
		return -(decimal+1)

	else:
		for i in range(len(binary)):
			decimal = decimal + int(binary[i])*pow(2,(len(binary)-(i+1)))
		return decimal
		

 
def position():
	s = ser.read(26)
	#for  i in s:
	s1 =list(map(bin,bytearray(s)))
	N = binarytodecimal(s1,17)
	E = binarytodecimal(s1,21)
	D = binarytodecimal(s1,25)
	print(N,E,D)
	
	return E,N,D

def Distance(E,N):

	return math.sqrt(E*E+N*N)


ser.close()
	
'''	
while(1):
    y,x,z=position()
    polar(float(y),float(x)) # need to use this polar mode RTK FIX mode other wise over flow error 
    time.sleep(0.5)
    
ser.close()
'''