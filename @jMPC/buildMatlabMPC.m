function nMPC = buildMatlabMPC(J,simopts)
%Build a MATLAB MPC Toolbox Object from a jMPC Object
%
%   Called By jMPC\sim

%   Jonathan Currie (C)
%   Control Engineering 2011

check_mpc();

if(isa(simopts.Plant,'jNL')), error('Nonlinear Models cannot be simulated using the jMPC -> MPC Toolbox interface'); end
if(J.sizes.n_out ~= J.sizes.nq_out), error('Uncontrolled outputs cannot be simulated using the jMPC -> MPC Toolbox interface'); end
if(isa(J.Model.A,'single')), error('Only Double Precision Simulations can be Run using the jMPC -> MPC Toolbox interface'); end

%Build Constraints Structures
n_in = J.sizes.n_in;
n_out = J.sizes.n_out;
MV(n_in).Min = []; MV(n_in).Max = []; MV(n_in).RateMin = []; MV(n_in).RateMax = [];
OV(n_out).Min = []; OV(n_out).Max = [];

con = J.constraints;
if(con.delucon)
    for i = 1:n_in
        MV(i).RateMin = -con.u(i,3);
        MV(i).RateMax = con.u(i,3);
    end
end
if(con.ucon)
    for i = 1:n_in
        MV(i).Min = con.u(i,1);
        MV(i).Max = con.u(i,2);
    end
end
if(con.ycon)
    for i = 1:n_out
        OV(i).Min = con.y(i,1);
        OV(i).Max = con.y(i,2);
    end
end

%Build Weights Structure
Weights.MV = zeros(1,J.sizes.n_in);
Weights.MVRate = J.QP.uwt';
Weights.OV = J.QP.ywt';

if(~isempty(con.slack))
    error('soft constraints not supported');
    Weights.ECR = J.constraints.slack(1);
else
    Weights.ECR = 1e6;
end

%Build MATLAB MPC Toolbox Object
nMPC = mpc(ss(simopts.Plant),J.Model.Ts,J.Np,J.Nc,Weights,MV,OV);

%Set Optimizer Properties
nMPC.Optimizer.MaxIter = J.mpcopts.QPMaxIter;
if(J.mpcopts.QPVerbose)
    nMPC.Optimizer.Trace = 'on';
end

end

function check_mpc()
%Check Control Systems Toolbox is installed
    c = ver('mpc');
    if(isempty(c))
        error('The MATLAB MPC Toolbox must be installed');
    end
end