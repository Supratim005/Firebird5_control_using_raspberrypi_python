import serial
import io
ser=serial.Serial('/dev/ttyACM0',19200)



def main():
	s = ser.read(26)
	#for  i in s:
	s1 = list(map(bin,bytearray(s)))
	N = bintodec(s1,4)
	#E = bintodec(s1,21)
	#D = bintodec(s1,25)
	print(N)






def bintodec(k,l):
	value = 0
	binary = []
	for p in range(4):
		no_zero_add = 10 - len((k[l-p])) # No zeros need to be add to make it 8 bit
		for p in range(no_zero_add):
			binary.append(0)
		for q in range(len(k[l-p])-2):
				binary.append(k[l-p][q+2]) # excluding 0b
				
	for i in range(len(binary)):
        
        digit = binary.pop()
        if digit == '1':
            value = value + pow(2, i)

	

 

main()


ser.close()