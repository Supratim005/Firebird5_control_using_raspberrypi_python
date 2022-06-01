import board
import adafruit_bno055
i2c = board.I2C()
#sensor = adafruit_bno055.BNO055_I2C(i2c)

def heading(sensor):
    #sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    return sensor.euler[0]*(22/1260)
