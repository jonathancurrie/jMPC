function xdot = nl_servo(t,x,u,param)
% Non Linear Position Servomechanism

% Reference: Model Predictive Control Toolbox 3, User's Guide, MATLAB

% x1 = theta
% x2 = theta_dot
% x3 = torque
% x4 = torque_dot

%Assign Parameters
[k0,kT,JM,JL,p,BM,BL,R] = param{:};     

xdot(1,1) = x(2);
xdot(2,1) = -(k0/JL)*(x(1)-x(3)/p)-(BL/JL)*x(2);
xdot(3,1) = x(4);
xdot(4,1) = (kT/JM)*((u-kT*x(4))/R)-(BM*x(4))/JM+(k0/(p*JM))*(x(1)-x(3)/p);