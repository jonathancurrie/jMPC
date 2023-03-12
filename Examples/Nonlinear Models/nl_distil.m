function xdot = nl_distil(t,x,u,param) %#ok<INUSL>
% Nonlinear Binary Distillation Column

% Reference: J. Hahn and T.F. Edgar (2002), An improved method for 
% nonlinear model reduction using balancing of empirical gramians, 
% Computers and Chemical Engineering, 26, pp. 1379-1397.

%Inputs
% u1 = fFeed - Feed Flowrate [mol/min]
% u2 = aFeed - Feed Mole Fraction
% u3 = RR - Reflux Ratio

%States
% x1 = Reflux Drum Liquid Mole Fraction of A
% x2 = Tray 1 Liquid Mole Fraction of A
% .
% x(FeedTray) = Liquid Mole Fraction of A on Feed Tray
% .
% x(n-1) = Tray n Liquid Mole Fraction of A
% x(n) = Reboiler Liquid Mole Fraction of A

%Equations
% x1_dot = 1/aCond*V*(y2-x1)
% x[2..feedTray-1]_dot = 1/aTray*(L*(x[n-1]-x[n])-V*(y[n]-y[n+1]))         e.g. x2_dot = 1/aTray*(L*(x1-x2)-V*(y2-y3))
% x[feedTray]_dot = 1/aTray*(fFeed*aFeed + L*x[feedTray-1] - FL*x[feedTray] - V*(y[feedTray]-y[feedTray+1]))
% x[feedTray+1..nTrays-1]_dot = 1/aTray*(FL*(x[n-1]-x[n])-V(y[n]-y[n+1]))  e.g. x18_dot = 1/aTray*(FL*(x17-x18)-V*(y18-y19))
% x[nTray]_dot = 1/aReb*(FL*x[nTray-1] - (fFeed-D)*x[nTray] - V*y[nTray])

%Additional Equations
% L = RR * D        
% FL = ffeed + L
% V = FL + D
% rVol = (y*(1-x))/((1-y)*x)
% x[n] + y[n] = 1


%Assign Parameters
[FT,rVol,aTray,aCond,aReb] = param{:};
xdot = NaN(size(x));
fFeed = u(1);
aFeed = u(2);
RR = u(3);

% Additional Eqs
D = 0.5*fFeed;              % Distillate Flowrate [mol/min]
L = RR*D;                   % Flowrate of the Liquid in the Rectification Section [mol/min]
V = L+D;                    % Vapor Flowrate in the Column [mol/min]
FL = fFeed+L;               % Flowrate of the Liquid in the Stripping Section [mol/min]
% Determine Vapour Flows + Delta Flows
y = x*rVol./(1+(rVol-1).*x);  % Assume constant relative volatility to determine vapour flows
dx = -diff(x); dy = -diff(y); % Difference molar flow rates

%Calculate flowrates on each tray
xdot(1,1) = 1/aCond*V*(y(2)-x(1));                                          % Reflux Drum 
xdot(2:FT-1,1) = (L*dx(1:FT-2) - V*dy(2:FT-1))/aTray;                       % Top section 
xdot(FT,1) = 1/aTray*(fFeed*aFeed+L*x(FT-1)-FL*x(FT)-V*(y(FT)-y(FT+1)));    % Feed tray 
xdot(FT+1:end-1) = (FL*dx(FT:end-1)-V*dy(FT+1:end))/aTray;                  % Bottom section 
xdot(end) = (FL*x(end-1)-(fFeed-D)*x(end)-V*y(end))/aReb;                   % Reboiler
end 