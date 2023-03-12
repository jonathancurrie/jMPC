function xdot = nl_quadtank(t,x,u,param) %#ok<INUSL>
% Nonlinear Quadruple Tank Model

% Reference: J. Åkesson (2006) MPCtools 1.0 Reference Manual, Lund
% Institute of Technology

%Inputs
% u1 = Pump 1 voltage [V]
% u2 = Pump 2 voltage [V]

%States
% x1 = Tank 1 Level (Bottom Left) [cm]
% x2 = Tank 2 Level (Bottom Right) [cm]
% x3 = Tank 3 Level (Top Left) [cm]
% x4 = Tank 4 Level (Top Right) [cm]

%Equations
% x1_dot = -a1/A2*sqrt(2*g*x1)+a3/A1*sqrt(2*g*x3)+(g1*k1)/A1*u1
% x2_dot = -a2/A2*sqrt(2*g*x2)+a4/A2*sqrt(2*g*x4)+(g2*k2)/A2*u2
% x3_dot = -a3/A3*sqrt(2*g*x3)+((1-g2)*k2)/A4*u2
% x4_dot = -a4/A4*sqrt(2*g*x4)+((1-g1)*k1)/A4*u1


%Assign Parameters
[A1,A2,A3,A4,a1,a2,a3,a4,g,g1,g2,k1,k2] = param{:};

xdot(1,1) = -a1/A1*sqrt(2*g*x(1))+a3/A1*sqrt(2*g*x(3))+(g1*k1)/A1*u(1);
xdot(2,1) = -a2/A2*sqrt(2*g*x(2))+a4/A2*sqrt(2*g*x(4))+(g2*k2)/A2*u(2);
xdot(3,1) = -a3/A3*sqrt(2*g*x(3))+((1-g2)*k2)/A3*u(2);
xdot(4,1) = -a4/A4*sqrt(2*g*x(4))+((1-g1)*k1)/A4*u(1);