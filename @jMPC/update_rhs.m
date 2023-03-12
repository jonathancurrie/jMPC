function [b,f] = update_rhs(J,del_xm,u,setp,mdist,k)
%Update RHS of Quadratic Programming problem
%
%   Called By jMPC\mpcsolve

%   Jonathan Currie (C)
%   AUT University 2011

% %Update quadprog f vector
len = size(setp,1); if(k > len); k = len; warning('jMPCToolbox:SETP','Setpoint Shorter Than Expected - Copying Last Row'); end
y0 = J.pred.Fq*del_xm(J.index.isq_out) + J.pred.Phiqv*mdist'; %add measured disturbance for prediction
if(J.mpcopts.LookAhead) %Actual setpoint projection
    setp = setp(k:k+J.Np-1,:)';
    setp = setp(:);
else % Constant setpoint projection
    setp = J.QP.S*setp(k,:)';
end
f = -J.QP.PhiTQ*(setp - y0);
%If soft constrained the final decision variable(s) are the slack variables,
%augment zeros in f
if(J.constraints.soft.no)
    f = [f; zeros(J.constraints.soft.no,1)];
end

if(~J.constraints.uncon) %Constrained MPC
    %Distribute structure variables
    idelu = J.constraints.indices.idelu;
    iumin = J.constraints.indices.iumin; 
    iumax = J.constraints.indices.iumax; 
    iymin = J.constraints.indices.iymin; 
    iymax = J.constraints.indices.iymax;

    bdyn = [];
    %Input Rate Constraints
    if(J.constraints.delucon), bdyn = zeros(2*sum(idelu),1); end
    %Input Constraints
    if(J.constraints.ucon)
        del = J.constraints.Tin*u;
        if(any(iumin)), bdyn=[bdyn;del(iumin,:)];end
        if(any(iumax)), bdyn=[bdyn;-del(iumax,:)];end
    end
    %Output Constraints
    if(J.constraints.ycon)
        %Full Output Prediction
        y0 = J.pred.F*del_xm + J.pred.Phiv*mdist';
        if(any(iymin)), bdyn=[bdyn;y0(iymin,:)]; end
        if(any(iymax)), bdyn=[bdyn;-y0(iymax,:)]; end
    end    
    %Update Constraints RHS
    b = J.constraints.bcon+bdyn;
else
    b = 0;
end

%Scale as required
if(J.mpcopts.ScaleSystem)
    f = J.QP.Hscale.*f;
    if(~J.constraints.uncon)
        b = J.QP.Ascale.*b;
    end
end

end