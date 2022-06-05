import board
import adafruit_bno055
i2c = board.I2C()
#sensor = adafruit_bno055.BNO055_I2C(i2c)

def heading(sensor,theta):
    #sensor = adafruit_bno055.BNO055_I2C(i2c)
    if theta-result,sensor.euler[0]<0:
        result=abs(theta-result,sensor.euler[0])

    else if theta-result,sensor.euler[0]>0:
        result=-abs(theta-result,sensor.euler[0])

    else if theta=0 and result,sensor.euler[0]<360 and result,sensor.euler[0]>180:
        result=
    
    return result,sensor.euler[0]*(22/1260)
