function display(J)
%Display Model Information

fprintf('-- jNL Nonlinear ODE Object --\n')
fprintf('State Function:  %s()\n',func2str(J.nl_model)); 
if(isa(J.C,'function_handle'))
    fprintf('Output Function: %s()\n',func2str(J.C));
else
    fprintf('Output Function: C\n');
end
switch(J.odesolver)
    case 0; odestr = 'ode45';
    case 1; odestr = 'ode23s';
    case 2; odestr = 'ode15s';
end
fprintf('ODE Solver: %s\n',odestr);
fprintf('States: %1d\n',length(J.x0));
fprintf('Parameters: %1d\n',length(J.param));            