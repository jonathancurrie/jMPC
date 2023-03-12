function [optdel_u,u] = saturate_inputs(optdel_u,u,n_in,cons)
%Saturate Delta U and U Inputs as per constraints
%
%   Called By jMPC/sim/simMPC

%   Jonathan Currie (C)
%   AUT University 2010

%Check if Problem is Constrained
if(cons.uncon || isempty(cons.u))
    %Sum Input Increment
    u = u + optdel_u;
else
    %Saturate Rate of Change of Input
    for i = 1:n_in
        if(optdel_u(i) > cons.u(i,3))
            optdel_u(i) = cons.u(i,3);
        elseif(optdel_u(i) < -cons.u(i,3))
            optdel_u(i) = -cons.u(i,3);
        end
    end
    %Sum Input Increment
    u = u + optdel_u; 
    %Saturate Inputs
    for i = 1:n_in
        if(u(i) > cons.u(i,2))
            u(i) = cons.u(i,2);
        elseif(u(i) < cons.u(i,1))
            u(i) = cons.u(i,1);
        end
    end
end

end

