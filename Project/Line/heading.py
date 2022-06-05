import board
import adafruit_bno055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

def heading(theta):
    #sensor = adafruit_bno055.BNO055_I2C(i2c)
    if sensor.euler[0]-theta>5.238095238:
        #h=-heading.heading(neg)
        neg=1

    elif sensor.euler[0]-theta<-5.238095238:

        #h=-heading.heading(neg)
        neg=2


    if neg==1:
        h=-sensor.euler[0]
        if h>-(22/7):
            neg=0

    elif neg ==2:
        h=sensor.euler[0]
        if h<(22/7):
            neg=0


    if(sensor.euler[0]<=180 and sensor.euler[0]>0) and neg==0:
        return -(sensor.euler[0])*(22/1260)
    elif(sensor.euler[0]>180 and sensor.euler[0]<=360) and neg ==0:
        return (360-sensor.euler[0])*(22/1260)

    elif sensor.euler[0]==0 or sensor.euler[0]==360:
        return 0

    elif neg==1:
        return sensor.euler[0]*(22/1260)

    elif neg==2:
        return (360-sensor.euler[0])*(22/1260)
    
