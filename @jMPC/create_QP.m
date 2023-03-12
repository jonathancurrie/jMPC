function [QP,state_est] = create_QP(Model,Phi,Np,Nb,uwt,ywt,con,Kest,sizes,opts)
%Create Quadratic Programming Structure
%
%   Called By buildMPC

%   Jonathan Currie (C)
%   Control Engineering 2011

states = sizes.states;
nm_in = sizes.nm_in;
n_out = sizes.n_out;
nq_out = sizes.nq_out;
nm_out = sizes.nm_out;

%Create weight matrix (weights are squared)
[r,c] = size(ywt); if(c > r); ywt = ywt'; end
[r,c] = size(uwt); if(c > r); uwt = uwt'; end
QP.Q = diag(repmat(ywt.*ywt,Np,1)); 
QP.R = diag(repmat(uwt.*uwt,Nb,1)); 

%If infinite-horizon (dual mode), add terminal weight)
if(opts.DualMode)
    if(isempty(which('dare.m')))
        jmpcwarn('jMPC:TWt','The Control Systems Toolbox must be installed for Dual-Mode MPC - ignoring option.');
    else
        Qt = dare(Model.A,Model.B,diag([repmat(ywt,states,1).^2;ones(n_out,1)]),diag(uwt.^2));
    end
end

%Create estimator gain matrix if not specified
if(isempty(Kest))
    state_est.Kest=[zeros(states,nm_out); eye(nm_out)];
    state_est.Est = 0;
    state_est.StdKest = [];
else
    state_est.Kest = [Kest; eye(nm_out)];
    state_est.Est = 1;
    state_est.StdKest = Kest;
end

%Create state estimator multiplier matrix
if(n_out ~= nm_out)
    Kest = [state_est.Kest(1:states,:) zeros(states,n_out-nm_out)]; %reshape
    Kest = [Kest; eye(n_out)];
else
    Kest = state_est.Kest;
end
state_est.IKC = eye(states+n_out)-Kest*Model.C;

%Create Prediction Update Matrix for f
QP.Phi = Phi;
QP.PhiTQ = Phi'*QP.Q;
%Form setpoint expansion matrix
QP.S = repmat(eye(nq_out),Np,1);
%Construct QP matrix
B = QP.PhiTQ*Phi+QP.R;
QP.H = (B+B')/2; %ensures symmetric

%If soft constrained augment slack variable(s) as the final decision variable(s)
if(con.soft.no) 
    QP.H = [QP.H zeros(Nb*nm_in,con.soft.no); zeros(con.soft.no,Nb*nm_in) diag(con.slack(con.soft.ind))];
end

[R,exitflag] = chol(QP.H);
if(exitflag)
    error('QP H Matrix Is Not Positive Definite');
else
    QP.R = R;
end
