import serial
import io
import math
from pyubx2 import UBXReader





def distance(E,N):
    return math.sqrt(E*E+N*N)

 
def position():
	ser=serial.Serial('/dev/ttyACM0',115200)
	ubr = UBXReader(ser, protfilter=2)
	(raw_data,_)=ubr.read()
	msg=UBXReader.parse(raw_data)
	E=msg.relPosE
	N=msg.relPosN
	D=msg.relPosD
	Distance=distance(E,N)
	#print(msg.relPosE,msg.relPosN)
	#print(Distance)
	return E/100,N/100,Distance
	ser.close()
	

l,m,y=position()
print(l*100,m*100,y)