import motion as pi 
import gps
import casadi as ca
import heading
import math
import time



#i2c = board.I2C()
#imu = adafruit_bno055.BNO055_I2C(i2c)

R1=0.05
R2=0.18



def vehicle(step_horizon, t0, u,sensor,theta):
    u=ca.floor(u)
    pi.velocity(int(u[1,0]),int(u[0,0]),1)
    time.sleep(0.97)
    pi.stop()
    
    x,y,_ = gps.position()
    h = theta-heading.heading(sensor)

    next_state = ca.vertcat(
    ca.horzcat(x), # East
    ca.horzcat(y), # North
    ca.horzcat(h)  # heading angle
    )

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0
