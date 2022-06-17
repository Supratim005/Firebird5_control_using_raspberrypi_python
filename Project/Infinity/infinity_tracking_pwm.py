import time
main_loop = time.time()     # return time in sec
import math
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
import vehicle_pwm
import board
import adafruit_bno055
import calibration
import motion as pi
pi.serial_open() # To serial access the avr board

'''
import serial
import io
import motion as pi 
import actuator
import csv
import board
import adafruit_bno055
import keyboard
i2c = board.I2C()

#===========================================================================================================================
#sensor interface

IMU = adafruit_bno055.BNO055_I2C(i2c)
GPS=serial.Serial('/dev/ttyACM0',19200) # For the GPS'''

#============================================================================================================================





#==============================================================Simulation variable==========================================================================================================================

step_horizon = 1    #sampling freq
N = 10              # number of look ahead steps
sim_time = 76      # simulation time

t_tra=np.arange(0,sim_time+N,step_horizon)

#============================================================input=====================================================================================================================
#input
pi=math.pi
sin=np.sin
cos=np.cos
atan2=np.arctan2
sqrt=math.sqrt
#x_target=1.1+0.7*sin((2*pi/200)*t_tra);
#y_target=0.8+0.7*sin((4*pi/200)*t_tra);
#theta_target=np.unwrap(atan2(2*pi*cos((pi*t_tra)/50), pi*cos((pi*t_tra)/100)));
x_target=1.1+0.5*sin((2*pi/75)*t_tra);
y_target=0.7+0.5*sin((4*pi/75)*t_tra);
theta_target=np.unwrap( atan2(2*pi*cos((4*pi*t_tra)/75), pi*cos((2*pi*t_tra)/75)));
#====================================================================================================================================================================================

#============================================================Sytem variables========================================================================================================================
# setting matrix_weights' variables

Q_x = 80000000000000000000
Q_y = 80000000000000000000
Q_theta = 30000000000
R1 = 8
R2 = 1

'''
Q_x = 60000
Q_y = 10000
Q_theta = 30000
R1 = 0.01
R2 =0.0001
'''
r=0.05/2 # radious
l=0.18 # base length


'''
Bellow
x_init = 1.10
y_init = 0.90
theta_init = pi/4
'''

pwm_r_max = 255; pwm_r_min= 110;
pwm_l_max = 247; pwm_l_min = 100;





# restruture the output
def shift_timestep(step_horizon, t0, state_init, u, f):
    u=ca.floor(u)
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


def DM2Arr(dm):
    return np.array(dm.full())


# state symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()

# control symbolic variables

v = ca.SX.sym('vpwm_r')
omega = ca.SX.sym('pwm_l')
controls = ca.vertcat(
    v,
    omega
)
n_controls= controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N)

# coloumn vector for storing initial state and target state
P = ca.SX.sym('P', n_states + N*(n_states+n_controls))

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_x, Q_y, Q_theta)

# controls weights matrix
R = ca.diagcat(R1, R2)



# discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
phi = ca.vertcat(
    ca.horzcat(cos(theta),  0),
    ca.horzcat(sin(theta),  0),
    ca.horzcat(         0, 1)
)

rl = ca.vertcat(
    ca.horzcat(r/2 , r/2),
    ca.horzcat(r/l ,-r/l )
)

#con= 3.8/255 #( 3.8 rad/sec /255)
con=ca.vertcat(
    ca.horzcat(4/145),
    ca.horzcat(4/147)
)

con1=ca.vertcat(
    ca.horzcat(110),
    ca.horzcat(100)
)



# Euler discretization
#RHS = con*(controls-con1)
RHS=phi @ (rl@(con * (controls-con1)))

#print(RHS)

f = ca.Function('f', [states, controls], [RHS])


cost_fn = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation

i=1;
# runge kutta
for k in range(0,N):
    st = X[:, k]
    con = U[:, k]
    cost_fn = cost_fn \
        + (st - P[(4*i+k)-1:(4*i+k)+2]).T @ Q @ (st - P[(4*i+k)-1:(4*i+k)+2]) \
        + (con-P[(4*i+k)+2:(4*i+k)+4]).T @ R @ (con-P[(4*i+k)+2:(4*i+k)+4])
    st_next = X[:, k+1]
    k1 = f(st, con)
    k2 = f(st + step_horizon/2*k1, con)
    k3 = f(st + step_horizon/2*k2, con)
    k4 = f(st + step_horizon * k3, con)
    #st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    f_val=f(st,con)
    st_next_euler=st+(step_horizon*f_val)
    g = ca.vertcat(g, st_next - st_next_euler)
    i=i+1


OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)

nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))



lbx[0: n_states*(N+1): n_states] = -ca.inf     # X lower bound
lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = ca.inf      # X upper bound
ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

'''

lbx[0: n_states*(N+1): n_states] = 0.5     # X lower bound
lbx[1: n_states*(N+1): n_states] = 0.1     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = 1.6      # X upper bound
ubx[1: n_states*(N+1): n_states] = 1.3     # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

'''


lbx[3*(N+1):3*(N+1)+2*N:2] = pwm_r_min                 
lbx[3*(N+1)+1:3*(N+1)+2*N:2] = pwm_r_min                 

ubx[3*(N+1):3*(N+1)+2*N:2] = pwm_r_max                 
ubx[3*(N+1)+1:3*(N+1)+2*N:2] = pwm_l_max                  


args = {
    'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
    'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
    'lbx': lbx,
    'ubx': ubx
}
#========================================problem setup =======================================================================================================

#========================================simulation init=======================================================================================================
t0 = 0
#x_init = 1.5
#y_init = 0.90
#theta_init=.88 #(50 digree)
#state_init = ca.DM([x_init, y_init, theta_init]) 
state_init = calibration.calibration()        # initial state
theta_init=state_init[2]
theta=state_init[2]

# xx = DM(state_init)
t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))  # initial control
X0 = ca.repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])

p = ca.DM.zeros(n_states + N*(n_states+n_controls),1)

xx=np.zeros([3,int(sim_time/step_horizon)])
cat_controls=np.zeros([2,int(sim_time/step_horizon)])
cat_states= np.zeros([3,int(sim_time/step_horizon)])
target_states= np.zeros([3,int(sim_time/step_horizon)])
neg=0

if __name__ == '__main__':

    
    
    while(mpc_iter * step_horizon < sim_time):
        t1 = time.time()

        current_time=mpc_iter

        p[0:3]= state_init

        j=1
        for k in range(0,N):

            t_predict=current_time+k-1

            x_ref=x_target[t_predict]

            y_ref=y_target[t_predict]

            theta_ref=theta_target[t_predict]

            u_ref=sqrt( (pi**2*cos((2*pi*t_predict)/75)**2)/5625 + (4*pi**2*cos((4*pi*t_predict)/75)**2)/5625)
            
            omega_ref=( -((8*pi**3*cos((2*pi*t_predict)/75)*sin((4*pi*t_predict)/75))/421875 - 
                (4*pi**3*cos((4*pi*t_predict)/75)*sin((2*pi*t_predict)/75))/421875)/((pi**2*cos((2*pi*t_predict)/75)**2)/5625 + 
                (4*pi**2*cos((4*pi*t_predict)/75)**2)/5625) )

            right_pwm_ref= math.floor((158/4)*((1/(2*r))*(2*u_ref+l*omega_ref))+97) # in pwm
            left_pwm_ref= math.floor((156/4)*((1/(2*r))*(2*u_ref-l*omega_ref))+91) # in  pwm


            p[(4*j+k)-1:(4*j+k)+2]=[x_ref, y_ref, theta_ref]

            p[(4*j+k)+2:(4*j+k)+4]=[right_pwm_ref, left_pwm_ref]
            j=j+1



        args['p'] = ca.vertcat(p)

        # optimization variable current state

        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )


        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N) # send only the 1st point u[:,0]


        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)


        t = np.vstack((
            t,
            t0
        ))


        #t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)


        t0, state_init, u0,theta,neg= vehicle_pwm.vehicle(step_horizon, t0, u,theta ,neg)
        #print("x:",state_init[0],"y:",state_init[1],"theta:",state_init[2]*(180/pi))
        #print("x_ref:",x_ref,"y_ref:",y_ref,"theta_ref:",theta_ref*(180/pi))


        xx[:,mpc_iter]=state_init.T
        cat_controls[:,mpc_iter] = ca.DM.full(u0[:,0]).T
        cat_states[:,mpc_iter] = state_init.T
        target_states[0,mpc_iter]= x_target[mpc_iter]
        target_states[1,mpc_iter]= y_target[mpc_iter]
        target_states[2,mpc_iter]= theta_target[mpc_iter]


        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time.time()
        times = np.vstack((
            times,
            t2-t1
        ))

        mpc_iter = mpc_iter + 1

    main_loop_time = time.time()

  

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')

    MSE = np.square(np.subtract(xx[0],x_target[0:sim_time])).mean()+np.square(np.subtract(xx[1],y_target[0:sim_time])).mean() 
    +np.square(np.subtract(xx[2],theta_target[0:sim_time])).mean() 
    RMSE = math.sqrt(MSE)
    print("Root Mean Square Error:\n",RMSE)

    np.savetxt("/home/pi/Firebird5_control_using_raspberrypi_python/Project/Infinity/States.csv", cat_states.T , delimiter="," , header='x,y,theta',comments='')
    np.savetxt("/home/pi/Firebird5_control_using_raspberrypi_python/Project/Infinity/target_states.csv", target_states.T , delimiter="," , header='x,y,theta',comments='')

    plt.figure(1)
    plt.plot(xx[0],xx[1],x_target[0:sim_time],y_target[0:sim_time])
    location = 0 # For the best location
    legend_drawn_flag = True 
    plt.legend(["Actual", "Target"], loc=0, frameon=legend_drawn_flag)
    plt.ylabel('Y(meter)')
    plt.xlabel('X(meter)')
    plt.suptitle("Tracking")
    plt.savefig('/home/pi/Firebird5_control_using_raspberrypi_python/Project/Infinity/Tracking.png')

    #====================control============================
    plt.figure(2)
    plt.subplot(121)
    plt.suptitle("Control Signal")
    plt.plot(cat_controls[1], color="orange")
    plt.xlabel('Left_pwm')
    plt.subplot(122)
    plt.plot(cat_controls[0], color="yellow")
    plt.xlabel('Right_pwm')
    plt.savefig('/home/pi/Firebird5_control_using_raspberrypi_python/Project/Infinity/controls.png')

    #=======================States=============================
    n=np.arange(1,sim_time+1)
    plt.figure(3)
    plt.subplot(311)
    plt.plot(n,cat_states[0],n,x_target[0:sim_time])
    plt.ylabel('X(meter)')
    plt.subplot(312)
    plt.plot(n,cat_states[1],n,y_target[0:sim_time])
    plt.ylabel('Y(meter)')
    plt.subplot(313)
    plt.plot(n,cat_states[2],n,theta_target[0:sim_time])
    plt.ylabel('Heading(radian)')
    plt.xlabel('time (sec)')
    location = 0
    legend_drawn_flag = True 
    plt.legend(["Actual", "Target"], loc=0, frameon=legend_drawn_flag)
    plt.suptitle("States")
    plt.savefig('/home/pi/Firebird5_control_using_raspberrypi_python/Project/Infinity/States.png')
    plt.show()
