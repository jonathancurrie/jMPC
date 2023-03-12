function display(Model)
%Display model information

[states,n_in] = size(Model.B);
n_dist = length(Model.meas_dist);
n_out = size(Model.C,1);
nm_out = length(Model.meas_out);
if(Model.isSingle)
    fprintf('-- jMPC State Space Object [Single Prec] --\n')
else
    fprintf('-- jMPC State Space Object --\n')
end
fprintf('States:  %1d\n',states);
if(n_dist > 0)
    fprintf('Manipulated Inputs:  %1d\n',n_in-n_dist);
    fprintf('Measured Disturbances: %1d\n',n_dist);
else
    fprintf('Inputs:  %1d\n',n_in);
end
if(nm_out ~= n_out)
    fprintf('Measured Outputs:   %1d\n',nm_out);
    fprintf('Unmeasured Outputs: %1d\n',n_out-nm_out);
else
    fprintf('Outputs: %1d\n',n_out);
end
if(Model.Ts > 0)
    fprintf('Sampling Time: %1.2g sec\n',Model.Ts);
else
    fprintf('Continuous Model\n');
end
if(Model.linearized)
    fprintf('Linearized about u: [%s], x: [%s]\n',num2str(Model.u_op'),num2str(Model.x_op'));
end