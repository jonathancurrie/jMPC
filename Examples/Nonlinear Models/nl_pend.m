function xdot = nl_pend(t,x,u,param) %#ok<INUSL>
% Nonlinear Inverted Pendulum on Cart

%Quanser IP02 Self-Erecting Inverted Pendulum User's Guide, 1996

%Inputs
% u1 = Force applied to the cart [N]

%States
% x1 = Position of the cart [m]
% x2 = Velocity of the cart [m/s]
% x3 = Angle of the pendulum from vertical [rad]
% x4 = Angular velocity of the pendulum [rad/s]

%Assign Parameters
[M,m,l,g] = param{:};

xdot(1,1) = x(2);
xdot(2,1) = (l*m*sin(x(3))*x(4)^2 + u(1) - g*m*cos(x(3))*sin(x(3)))/(M + m - m*cos(x(3))^2);
xdot(3,1) = x(4);
xdot(4,1) = -(l*m*cos(x(3))*sin(x(3))*x(4)^2 + u(1)*cos(x(3)) - g*m*sin(x(3)) - M*g*sin(x(3)))/(l*(M + m - m*cos(x(3))^2));




% %% OLD MODEL
% % Reference: David I Wilson, 3rd Assignment in Optimal Control, AUT
% % University
%
% %Equations
% % y_dot_dot = (f/m + l*theta^2 - g*sin(theta)*cos(theta))/(M/m + sin^2(theta))
% % theta_dot_dot = (-f/m*cos(theta) + (M+m)/m*g*sin(theta)-l*theta^2*sin(theta)*cos(theta))/(l*(M/m + sin^2(theta)))
% 
% %Assign Parameters
% [M,m,l,g] = param{:};
% 
% xdot(1,1) = x(2);
% xdot(2,1) = (u/m + l*x(3)^2*sin(x(3)) - g*sin(x(3))*cos(x(3)))/(M/m + sin(x(3))^2);
% xdot(3,1) = x(4);
% xdot(4,1) = (-u/m*cos(x(3))+ (M+m)/m*g*sin(x(3))-l*x(3)^2*sin(x(3))*cos(x(3)))/(l*(M/m + sin(x(3))^2));