function [Gs, Vop, nlheli, param, Kf, m_h, m_w, m_f, m_b, Lh, La, Lw, g, Je, Jt, Jp, Fg, Tg, Kt, Jm, Rm, K_EC_T, K_EC_P, K_EC_E] = setup_embed_heli()
%Return Quanser 3DOF Helictoper Model + Parameters

% Propeller force-thrust constant found experimentally (N/V)
Kf = 0.1188;
% Mass of the helicopter body (kg)
m_h = 1.15;
% Mass of counter-weight (kg)
m_w = 1.87;
% Mass of front propeller assembly = motor + shield + propeller + body (kg)
m_f = m_h / 2;
% Mass of back propeller assembly = motor + shield + propeller + body (kg)
m_b = m_h / 2;
% Distance between pitch pivot and each motor (m)
Lh = 7.0 * 0.0254;
% Distance between elevation pivot to helicopter body (m)
La = 26.0 * 0.0254;
% Distance between elevation pivot to counter-weight (m)
Lw = 18.5 * 0.0254;
% Gravitational Constant (m/s^2)
g = 9.81;    
% Travel, Pitch, and Elevation Encoder Resolution (rad/count)
K_EC_T = 2 * pi / ( 8 * 1024 );
K_EC_P = 2 * pi / ( 4 * 1024 );
K_EC_E = - 2 * pi / ( 4 * 1024 );
% Motor Armature Resistance (Ohm)
Rm = 0.83;
% Motor Current-Torque Constant (N.m/A)
Kt = 0.0182;
% Motor Rotor Moment of Inertia (kg.m^2)
Jm = 1.91e-6;

%Moment of inertia about elevation and travel axes (kg.m^2)
Je = 0.91; 
Jt = (2*m_f*La^2+2*m_f*Lh^2+m_w*Lw^2);%0.91;
%Moment of inertia about pitch axis (kg.m^2)
Jp = 0.0364; 

%Effective Differential Gravitational Torque due to Fg (Nm)
Tg = g*(Lw*m_w - La*m_f - La*m_b);
%Mass Differential about Elevation Axis (N)
Fg = Tg/La; 

%Build SS Model
A = zeros(6);
A( 1, 4 ) = 1;
A( 2, 5 ) = 1;
A( 3, 6 ) = 1;
A( 6, 2 ) = (2*m_f*La-m_w*Lw)*g/(2*m_f*La^2+2*m_f*Lh^2+m_w*Lw^2);
B = zeros(6,2);
B( 4, 1 ) = La*Kf/(m_w*Lw^2+2*m_f*La^2);
B( 4, 2 ) = La*Kf/(m_w*Lw^2+2*m_f*La^2);
B( 5, 1 ) = 1/2*Kf/m_f/Lh;
B( 5, 2 ) = -1/2*Kf/m_f/Lh;
C = zeros(3,6);
C( 1, 1 ) = 1;
C( 2, 2 ) = 1;
C( 3, 3 ) = 1;
D = zeros(3,2);
Gs = ss(A,B,C,D);
%Input Operating Point
uop = 0.5*(g*(Lw*m_w - La*m_f - La*m_b))/(La*Kf);
Vop = [uop;uop];
%NL Model & parameter array
nlheli = @nl_heliQ; %quanser version
param = {Je,La,Kf,Tg,Jp,Lh,Jt};






