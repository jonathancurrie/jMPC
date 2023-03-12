function xdot = nl_heli(t,x,u,param) %#ok<INUSL>
% Nonlinear 3DOF Helicopter Model

% Reference: J. Åkesson (2006) MPCtools 1.0 Reference Manual, Lund 
% Institute of Technology

%Inputs
% u1 = Motor 1 voltage [V]
% u2 = Motor 2 Voltage [V]

%States
% x1 = Elevation angle [rad]
% x2 = Elevation angular velocity [rad/s]
% x3 = Rotation angle [rad]
% x4 = Rotation angular velocity [rad/s]
% x5 = Pitch angle [rad]
% x6 = Pitch angular velcoity [rad/s]

%Equations
% theta_e_dot_dot = (Kf*la/Je)*(Vf+Vb)*cos(theta_p)-Tg/Je
% theta_r_dot_dot = -(Fg*la/Jt)*sin(theta_p)
% theta_p_dot_dot = (Kf*lh/Jp)*(Vf-Vb)


%Assign Parameters
[Je,la,Kf,Fg,Tg,Jp,lh,Jt] = param{:};

xdot(1,1) = x(2);
xdot(2,1) = (Kf*la/Je)*(u(1)+u(2))*cos(x(5))-Tg/Je;
xdot(3,1) = x(4);
xdot(4,1) = -(Fg*la/Jt)*sin(x(5));
xdot(5,1) = x(6);
xdot(6,1) = (Kf*lh/Jp)*(u(1)-u(2));