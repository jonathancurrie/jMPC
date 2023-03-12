function [x0,exit_flag] = jsolve_ss(fcn,u0,x0,param,odeS,verb) %#ok<INUSL>
% Solve for Steady State Operating Values
%
%   Called By jNL/linearize

%   Jonathan Currie (C)
%   Control Engineering 2011

global yold counter tol

%Construct Function Handle
f = eval(['@(t,x)' func2str(fcn) '(t,x,u0,param)']);
f2 = eval(['@(x)' func2str(fcn) '(0,x,u0,param)']);
%Setup
n = length(x0);
yold = inf*ones(n,1);
counter = 10;
tol = 1e-4; %must be less than reltol of solver
%Set ODE Options
odeopts = odeset('Events',@events);
       
%Solve For Steady State States at u0
if(verb)
    fprintf('Solving for Steady State about u = [%s] ...\n',num2str(u0'));
end

%Standard solve lengths (ode45 ode23s ode15s)
T = [3e3 3e3 5e3];

%Start searching with standard settings and the user chosen ode solver
[t,y] = odeSolve(f,T,x0,odeopts,odeS); %#ok<ASGLU>
exit_flag = 1;

%Failure 1
if(counter > 0) 
    %Try fsolve
    fx0 = y(end,:)'; %Take best guess so far
    if(exist('trnlsp','file') == 3)
        [y,rub,flag] = opti_fsolve(f2,fx0); %#ok<ASGLU> %Nonlinear equation solver      
    elseif(exist('fsolve.m','file') == 2)
        [y,rub,flag] = fsolve(f2,fx0,optimset('Display','off')); %#ok<ASGLU> %Nonlinear equation solver
    else
        error('No nonlinear equation solver available!\n\nEither supply the steady-state state vector (xss), download OPTI Toolbox, %s you will need the Optimization Toolbox.','or');
    end
    exit_flag = 2; 

    %Failure 2
    if(flag < 1)
        %Increase Tfinal and try again
        T = T + [2e3 3e3 4e3];
        counter = 10; yold = inf*ones(n,1);
        [t,y] = odeSolve(f,T,x0,odeopts,odeS); %#ok<ASGLU>
        exit_flag = 3;
        
        %Failure 3
        if(counter > 0)
            %Relax ode tolerance and have another go
            tol = 5e-3;
            counter = 10; yold = inf*ones(n,1);
            [t,y] = odeSolve(f,T,x0,odeopts,odeS); %#ok<ASGLU>
            exit_flag = 4;
            
            %Failure 4
            if(counter > 0)
                %Give up!
                exit_flag = -1;
            end
        end
    end
end
      
%Extract final solution
if(exit_flag == 2)
    x0 = y;
else
    x0 = y(end,:)';
end

if(exit_flag > 0 && verb)
    %Construct Summary Report
    sumStr = 'Solved using ';
    if(exit_flag == 2)
        sumStr = [sumStr 'fsolve'];
    else
        switch(odeS)
            case 0; sumStr = [sumStr 'ode45'];
            case 1; sumStr = [sumStr 'ode23s'];
            case 2; sumStr = [sumStr 'ode15s'];
        end
        switch(exit_flag)
            case 3; sumStr = [sumStr ' (Increased Tfinal to Solve)'];
            case 4; sumStr = [sumStr ' (Increased Tfinal & Relaxed SS Tolerance to Solve)'];
        end
    end

    %Print Summary
    fprintf([sumStr '\n']);      
    %Print Steady State
    fprintf('xss:');
    disp(x0');
end
    


function [value,isterminal,direction] = events(t,y) %#ok<INUSL>
%Check tolerance of solution for steady state
global yold counter tol ss_flag

c = norm(y-yold);
if(c < tol) %system is at our defined steady state
    counter = counter - 1; %subtract counter to test how long stays here
    ss_flag = 1; 
elseif(ss_flag) %tolerance broken, system moving again
    counter = 10; %reset counter
end
yold = y;

value = counter;
isterminal = 1; %stop integration if zero crossing achieved (i.e. counter < 0)
direction = -1;
    


function [t,y] = odeSolve(f,T,x0,odeopts,odeSolver)
%Function to evaluate ode solver to find steady state   
switch(odeSolver)
    case 0
        [t,y] = ode45(f,[0 T(1)],x0,odeopts); 
    case 1
        [t,y] = ode23s(f,[0 T(2)],x0,odeopts);
    case 2
        [t,y] = ode15s(f,[0 T(3)],x0,odeopts);
end 





