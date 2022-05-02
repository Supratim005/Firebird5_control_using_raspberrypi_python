clear all
clc
syms t_predict
%x=5* sin((2*pi/100)*t_predict).^3;
%y=4*cos((2*pi/100)*t_predict)-1.3*cos((2*pi/100)*2*t_predict)-0.6*cos((2*pi/100)*3*t_predict)-0.2*cos((2*pi/100)*4*t_predict);
%x=sin((2*pi/100)*t_predict);
%y=cos((2*pi/100)*t_predict);
x=1.1+0.7*sin((2*pi/200)*t_predict);
y=0.9+0.7*sin((4*pi/200)*t_predict);
z=(diff(x))^2 + (diff(y))^2  % u_ref=sqrt(z) 
omega= ((diff(diff(y))*diff(x) - diff(diff(x))*diff(y))/( diff(x)^2+diff(y)^2 )  )
theta= atan2(diff(y),diff(x))
