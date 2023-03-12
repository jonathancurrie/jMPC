function J = NLupdate(J,us,xs)
%Non Linear MPC Update Step
%
%   Called by jMPC/simNLMPC

%   Jonathan Currie
%   Control Engineering 2011


%Extract NL Model
NLModel = J.StdModel;

% %Re-Linearize about current operating point
% %Try use OPTI's mklJac if available
% try
%     if(verb); fprintf('Solving for A...'); end      
%     A = mklJac(@(x) NLModel.nl_model(t,x,us,NLModel.param), x_op);
%     if(verb); fprintf('Done\n'); end
% 
%     if(verb); fprintf('Solving for B...'); end
%     B = mklJac(@(u) NLModel.nl_model(t,xs,u,NLModel.param), u_op);
%     if(verb); fprintf('Done\n'); end
% catch
%     f = eval(['@(t,x)' func2str(NLModel.nl_model) '(t,x,us,NLModel.param)']);
%     f2 = eval(['@(t,u)' func2str(NLModel.nl_model) '(t,xs,u,NLModel.param)']);
%     A = numjac(f,1,xs,f(1,xs),1e-6,[]);
%     B = numjac(f2,1,us,f2(1,us),1e-6,[]);     
% end
% if(isa(NLModel.C,'function_handle'))
%     %Construct Function Handles
%     g = eval(['@(t,x)' func2str(NLModel.C) '(t,x,us,NLModel.param)']);
%     g2 = eval(['@(t,u)' func2str(NLModel.C) '(t,xs,u,NLModel.param)']);
%     C = numjac(g,1,xs,g(1,xs),1e-6,[]); 
%     D = numjac(g2,1,us,g2(1,us),1e-6,[]);
% else %already linear output function
%     C = NLModel.C; %#ok<*PROP>
%     D = zeros(size(C,1),length(us)); %jMPC does not use D matrix
% end
% Model = ss(A,B,C,D);
% Model.UserData.u_op = us;
% Model.UserData.x_op = xs;
% Model.UserData.y_op = Model.C*xs;

Model = linearize(NLModel,us,xs);

Model = augment(c2d(jSS(Model),J.Model.Ts));

%Build New Prediction Matrices
pred = J.prediction(Model,Model,J.Np,J.Nc,J.Nb,J.index);
QP = J.QP;
%Setup Constraints
%con = jMPC.setup_constraints(con,Phi,Np,Nb,n_in);
% 
%Create state update vector for f
QP.PhiTQ = pred.Phi'*QP.Q;
%Construct QP matrices
%Construct QP matrix
B = QP.PhiTQ*pred.Phi+QP.R;
QP.H = (B+B')/2; %ensures symmetric

%If soft constrained augment slack variable as the final decision variable
if(J.constraints.soft.no)
    sizes = J.sizes;
    QP.H = [QP.H zeros(sizes.Nb*sizes.nm_in,1); zeros(1,sizes.Nb*sizes.nm_in) diag(con.slack(con.soft.ind))];
end

%Perform System Scaling if Requested
if(J.mpcopts.ScaleSystem)
    Hfac = 1/max(max(QP.H(1:J.sizes.Nbm_in,1:J.sizes.Nbm_in)));
    QP.Hscale = [Hfac*ones(J.sizes.Nbm_in,1);ones(J.constraints.soft.no,1)];  
    QP.H = diag(QP.Hscale)*QP.H;
    [R,exitflag] = chol(QP.H);
    if(exitflag)
        jmpcwarn('jMPC:PosDef','QP H Matrix Is Not Positive Definite');
    else
        QP.R = R;
        J.QP = QP;
        J.pred = pred;
    end
else
    [R,exitflag] = chol(QP.H);
    if(exitflag)
        jmpcwarn('jMPC:PosDef','QP H Matrix Is Not Positive Definite');
    else
        QP.R = R;
        QP.Hscale = ones(sizes.Nb_in+con.soft.no,1);
        J.QP = QP;
        J.pred = pred;
    end    
end 
