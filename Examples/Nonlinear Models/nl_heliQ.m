function xdot = nl_heliQ(t,x,u,param) %#ok<INUSL>
% QUANSER Nonlinear 3DOF Helicopter Model

%Inputs
% u1 = Motor 1 voltage [V]
% u2 = Motor 2 Voltage [V]

%States
% x1 = Elevation angle [rad]
% x2 = Pitch angle [rad]
% x3 = Rotation angle [rad]
% x4 = Elevation angular velocity [rad/s]
% x5 = Pitch angular velcoity [rad/s]
% x6 = Rotation angular velocity [rad/s]

%Assign Parameters
[Je,La,Kf,Tg,Jp,Lh,Jt] = param{:};

xdot(1,1) = x(4); %elev vel
xdot(2,1) = x(5); %pit vel
xdot(3,1) = x(6); %rot vel

xdot(4,1) = (Kf*La/Je)*(u(1)+u(2))*cos(x(2))-Tg/Je; %elev eq
xdot(5,1) = (Kf*Lh/Jp)*(u(1)-u(2)); %pitch eq
xdot(6,1) = -(Kf*La/Jt)*(u(1)+u(2))*sin(x(2)); %rot eq

