
from time import time
main_loop = time()     # return time in sec
import math
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
#import vehicle

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
sim_time = 200      # simulation time

t_tra=np.arange(0,sim_time+N,step_horizon)

#============================================================input=====================================================================================================================
#input
pi=math.pi
sin=np.sin
cos=np.cos
atan2=np.arctan2
sqrt=math.sqrt
x_target=1.1+0.7*sin((2*pi/200)*t_tra);
y_target=0.9+0.7*sin((4*pi/200)*t_tra);
theta_target=np.unwrap(atan2(2*pi*cos((pi*t_tra)/50), pi*cos((pi*t_tra)/100)));

#====================================================================================================================================================================================

#============================================================Sytem variables========================================================================================================================
# setting matrix_weights' variables
Q_x = 60000
Q_y = 10000
Q_theta = 30000
R1 = 0.01
R2 =0.0001
r=0.05/2 # radious
l=0.18 # base length
x_init = 1.10
y_init = 0.90
theta_init = pi/4
pwm_r_max = 255; pwm_r_min= 0;
pwm_l_max = 255; pwm_l_min = 0;





# restruture the output
def shift_timestep(step_horizon, t0, state_init, u, f):
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
n_controls = controls.numel()


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

con= 3.8/255 #( 3.8 rad/sec /255)



# Euler discretization
RHS = phi @ rl @  con @ controls


# maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
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

lbx[0: n_states*(N+1): n_states] = -2     # X lower bound
lbx[1: n_states*(N+1): n_states] = -1     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = 2      # X upper bound
ubx[1: n_states*(N+1): n_states] = 2      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound


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
state_init = ca.DM([x_init, y_init, theta_init])        # initial state

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

if __name__ == '__main__':

    
    
    while(mpc_iter * step_horizon < sim_time):
        t1 = time()

        current_time=mpc_iter

        p[0:3]= state_init

        j=1
        for k in range(0,N):

            t_predict=current_time+k+1

            x_ref=x_target[t_predict]

            y_ref=y_target[t_predict]

            theta_ref=theta_target[t_predict]

            u_ref= sqrt( (49*pi**2*cos((pi*t_predict)/50)**2)/250000 + (49*pi**2*cos((pi*t_predict)/100)**2)/1000000 ) 

            omega_ref=((49*pow(pi,3)*cos((pi*t_predict)/50)*sin((pi*t_predict)/100))/50000000 - (49*pow(pi,3)*cos((pi*t_predict)/100)*sin((pi*t_predict)/50))/25000000)/((49*pow(pi,2)*cos((pi*t_predict)/50)**2)/250000 + (49*pi**2*cos((pi*t_predict)/100)**2)/1000000);

            p[(4*j+k)-1:(4*j+k)+2]=[x_ref, y_ref, theta_ref]

            p[(4*j+k)+2:(4*j+k)+4]=[u_ref, omega_ref]
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

      


        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))


        t = np.vstack((
            t,
            t0
        ))


        #t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)


        t0, state_init, u0 = vehicle.vehicle(step_horizon, t0, state_init, u, f)


        xx[:,mpc_iter]=state_init.T


        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        times = np.vstack((
            times,
            t2-t1
        ))

        mpc_iter = mpc_iter + 1

        time.sleep(0.5)



    main_loop_time = time()

    #plt.plot()

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    plt.plot(xx[0],xx[1],x_target,y_target)
    plt.show()