clear all
close all
clc



addpath('C:\Users\Supratim Dey\MATLAB Drive\casadi')
%addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

N =8; % prediction horizon
sim_tim =40;
T=1;
t_tra=0:T:sim_tim+N;

%x_tra=sin((2*pi/100)*t_tra);
%y_tra=cos((2*pi/100)*t_tra);
%theta_x=atan2(-pi*sin((pi*t_tra)/50), pi*cos((pi*t_tra)/50));

x_tra=1.1+0.7*sin((2*pi/200)*t_tra);
y_tra=0.9+0.7*sin((4*pi/200)*t_tra);
theta_x=atan2(2*pi*cos((pi*t_tra)/50), pi*cos((pi*t_tra)/100));



theta_tra=unwrap(theta_x);

%==========================================================
Q = zeros(3,3); Q(1,1) = 60;Q(2,2) =100;Q(3,3) = 30; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.8; R(2,2) = 0.05; % weighing matrices (controls)
x0 = [1.10; 0.90 ; pi/4];    % initial condition.
v_max = .075; v_min = -v_max;  
omega_max = .7; omega_min = -omega_max;
%===========================================================


rob_diam = 0.18; 





x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); 
omega = SX.sym('omega');
controls = [v;omega]; 
n_controls = length(controls);

%---------------------------------------------------------------------
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
 % i/p is control, o/p is states function is rhs

U = SX.sym('U',n_controls,N); % Decision variables (controls)

%P = SX.sym('P',n_states + n_states);
P = SX.sym('P',n_states + N*(n_states+n_controls));
% parameters (which include the initial state and the reference along the
% predicted trajectory (reference states and reference controls))

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [ ];  % constraints vector

%--------------------------------------------------------------------------------------------------------------------------------


st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    %obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    obj = obj+(st-P(5*k-1:5*k+1))'*Q*(st-P(5*k-1:5*k+1)) + ...
              (con-P(5*k+2:5*k+3))'*R*(con-P(5*k+2:5*k+3)) ; % calculate obj
    % the number 5 is (n_states+n_controls)
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 10000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -20; %state x lower bound % new - adapt the bound
args.ubx(1:3:3*(N+1),1) = 20; %state x upper bound  % new - adapt the bound
args.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 2; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;



xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

 % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
x_c=zeros(1,sim_tim / T);
y_c=zeros(1,sim_tim / T);
theta_c=zeros(1,sim_tim / T);
theta_cc=zeros(1,sim_tim / T);
l=1;
u=zeros(N,2);

while(mpciter < sim_tim / T) % new - condition for ending the loop
    
    current_time = mpciter;  %new - get the current time
    % args.p   = [x0;xs]; % set the values of the parameters vector
    %----------------------------------------------------------------------
    args.p(1:3) = x0; % initial condition of the robot postur
    for k = 1:N %new - set the reference to track
        t_predict = current_time + (k) ;% predicted time instant
        %======================================================================
        
        
        x_ref=x_tra(t_predict);
        y_ref=y_tra(t_predict);
        theta_ref=theta_tra(t_predict); 
        %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        %infinity
        u_ref =realsqrt( (49*pi^2*cos((pi*t_predict)/50)^2)/250000 + (49*pi^2*cos((pi*t_predict)/100)^2)/1000000 ); 
        %circle
        %u_ref=realsqrt( (pi^2*sin((pi*t_predict)/50)^2)/2500 + (pi^2*cos((pi*t_predict)/50)^2)/2500 );
        
        
        
        %#################################################################################
        
        %infinity
        omega_ref = ((49*pi^3*cos((pi*t_predict)/50)*sin((pi*t_predict)/100))/50000000 - (49*pi^3*cos((pi*t_predict)/100)*sin((pi*t_predict)/50))/25000000)/((49*pi^2*cos((pi*t_predict)/50)^2)/250000 + (49*pi^2*cos((pi*t_predict)/100)^2)/1000000);
 
        %circle
        %omega_ref =-((pi^3*sin((pi*t_predict)/50)^2)/125000 + (pi^3*cos((pi*t_predict)/50)^2)/125000)/((pi^2*sin((pi*t_predict)/50)^2)/2500 + (pi^2*cos((pi*t_predict)/50)^2)/2500);
        %square

        
        args.p(5*k-1:5*k+1) = [x_ref, y_ref, theta_ref];
        args.p(5*k+2:5*k+3) = [u_ref, omega_ref];
    end
    [x_c(l) , y_c(l)] = func(current_time) ;
    
 %theta_c(l)=atan2(2*pi*cos((pi*current_time)/25), pi*cos((pi*current_time)/50));
 
     
    
    %----------------------------------------------------------------------    
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:,mpciter+1) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    l;
    mpciter;
    mpciter = mpciter + 1;
    l=l+1;
end
main_loop_time = toc(main_loop);
average_mpc_time = main_loop_time/(mpciter+1);