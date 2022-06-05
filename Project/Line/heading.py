import board
import adafruit_bno055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

def heading(neg):
    #sensor = adafruit_bno055.BNO055_I2C(i2c)
    if(sensor.euler[0]<=180 and sensor.euler[0]>0) and neg==0:
        return -(sensor.euler[0])*(22/1260)
    elif(sensor.euler[0]>180 and sensor.euler[0]<=360) and neg ==0:
        return (360-sensor.euler[0])*(22/1260)

    elif neg==1:
        return sensor.euler[0]*(22/1260)

    elif neg==2:
        return (360-sensor.euler[0])*(22/1260)
    else:
        return 0
