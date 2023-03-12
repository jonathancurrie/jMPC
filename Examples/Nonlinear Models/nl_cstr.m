function xdot = nl_cstr(t,x,u,param) %#ok<INUSL>
% Nonlinear Continuously Stirred Reactor with single reaction A->B and
% energy balance

% Reference: M. Henson, D. Seborg (1997), Nonlinear Process Control,
% Prentice Hall PTR, page 5.

%Inputs
% u1 = CAf - Concentration of A in Feed [mol/m^3]
% u2 = Tf - Temperature of Feed [K]
% u3 = Tc - Temperature of Cooling Jacket [K]

%States
% x1 = CA - Concentration of A in reactor [mol/m^3]
% x2 = Tr - Reactor Temperature [K]

%Equations
% r = k0*exp(-E/(R*Tr))*CA
% x1_dot = q/V*(CAf-CA) - r
% x2_dot = q/V*(Tf-Tr) + (-H/(Cp*rho))*r + (U*A)/(Cp*rho*V)*(Tc-Tr)


%Assign Parameters
[q,V,k0,E,R,H,Cp,rho,UA] = param{:};

r = k0*exp(-E/(R*x(2)))*x(1);

xdot(1,1) = q/V*(u(1)-x(1)) - r;
xdot(2,1) = q/V*(u(2)-x(2)) + (H/(Cp*rho))*r + (UA)/(Cp*rho*V)*(u(3)-x(2));