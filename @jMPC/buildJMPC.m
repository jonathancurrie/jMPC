function MPCstruct = buildJMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
%Build a jMPC Model Predictive Controller
%
%   Called By jMPC Constructor

%   Jonathan Currie (C)
%   AUT University 2011

%Check for Nonlinear Simulation
if(isa(Model,'jNL'))
    Model = Model.lin_model;
end

%Collect sizes
[n_out,states] = size(Model.C);
n_in = size(Model.B,2);
nm_out = length(Model.meas_out);    %Number of measured outputs
nm_dist = length(Model.meas_dist);  %Number of measured disturbances
nm_in = n_in-nm_dist;               %Number of manipulated inputs

%Check & Correct Blocking Vector
if(size(Nc,1) > size(Nc,2))
    Nc = Nc';
end
Nb = length(Nc);
if(Nb > 1) %blocking moves
    sumNc = sum(Nc);
    if sumNc > Np
        con.warning{1} = 'Sum(Nc) > Np - Moves will be truncated to Np';
        nb = find(cumsum(Nc) > Np);
        if(Np-sum(Nc(1:nb-1)) > 0)
            Nc = [Nc(1:nb-1) Np-sum(Nc(1:nb-1))];
        else
            Nc = Nc(1:nb-1);
        end
    elseif sumNc < Np
        con.warning{1} = 'Sum(Nc) < Np - Moves will be extended to Np';
        Nc(Nb+1) = Np-sumNc;
    end
    Nb = length(Nc);
else
    Nb = Nc;
end

%Save measured outputs
imeas_out = Model.meas_out; 
index.imeas_out = imeas_out;
%Check for Unmeasured Outputs and remove operating points if so
if(nm_out ~= n_out)
    y_op = Model.y_op(imeas_out);
    %Create a vector of unmeasured output indices
    index.iumeas_out = 1:n_out;
    index.iumeas_out(imeas_out) = [];   
else
    y_op = Model.y_op;
    index.iumeas_out = [];
end
lin.y_op_full = Model.y_op;

%Controlled Outputs
iq_out = find(ywt ~= 0); 
[r,c] = size(iq_out); 
if(r > c); iq_out = iq_out'; end
index.iq_out = iq_out; %index of controlled outputs
index.isq_out = [1:states states+iq_out]; %index of controlled outputs + states within del_xm
nq_out = length(iq_out);
%If uncontrolled outputs, build separate model
if(nq_out ~= n_out)         
    C = Model.C(iq_out,:); %remove uncontrolled rows
    QPModel = jSS(Model.A,Model.B,C,Model.D,Model.Ts,Model.x0,Model.u_op,Model.x_op); %build new model for QP building
    ywt = ywt(iq_out); %remove weights
else
    QPModel = Model; %QP model is same as default Model
end

%Measured Disturbances
index.imeas_dist = Model.meas_dist;
index.iman_u = 1:size(Model.B,2); index.iman_u(index.imeas_dist) = [];

%State Estimation Index Vectors
index.iumeasp_out = index.iumeas_out + states;
index.ismeasp_out = [1:states index.imeas_out+states];
    
%Save Sizes Structure
sizes.states = states; sizes.n_out = n_out; sizes.n_in = n_in;
sizes.Nb_in = Nb*n_in; sizes.Np_out = Np*n_out; 
sizes.Nbm_in = Nb*nm_in;
sizes.nq_out = nq_out;      %Number of controlled outputs
sizes.nm_out = nm_out;      %Number of measured outputs
sizes.nm_dist = nm_dist;    %Number of measured disturbances
sizes.nm_in = nm_in;        %Number of manipulated inputs

%Linearized Model
if(Model.linearized)
    if(isfield(con,'u')) 
        con.u = con.u-Model.u_op(index.iman_u)*[1 1 0]; 
    end
    if(isfield(con,'y'))
        con.y = con.y-Model.y_op*[1 1];  
    end
    Model.x0 = Model.x0 - Model.x_op;
end
lin.u_op = Model.u_op;
lin.x_op = Model.x_op;
lin.y_op = y_op; 
lin.islin = Model.linearized;

%Check for initial input
if(~isempty(opts.InitialU))
    init_u = opts.InitialU;
else
    init_u = zeros(n_in,1); %purposely n_in as it gets indexed later (saves error check)
end

%Augment Output (adds Integrator)
x0 = Model.x0; %collect original states
sA = ssdata(Model);
Model = augment(Model);
QPModel = augment(QPModel);

%Form prediction matrices
pred = jMPC.prediction(Model,QPModel,Np,Nc,Nb,index);

%Setup Constraints
con = jMPC.SetupConstraints(con,pred.Phi,Np,Nb,nm_in,n_out,opts);
%Assign numeric index positions (for MEX + S Function)
index.mex.idelu = uint32(find(con.indices.idelu)-1);
index.mex.iumin = uint32(find(con.indices.iumin)-1);
index.mex.iumax = uint32(find(con.indices.iumax)-1);
index.mex.iymin = uint32(find(con.indices.iymin)-1);
index.mex.iymax = uint32(find(con.indices.iymax)-1);
index.mex.imeas_out = uint32(index.imeas_out-1)';
index.mex.iumeas_out = uint32(index.iumeas_out-1)';
index.mex.imeas_dist = uint32(index.imeas_dist-1)';
index.mex.iman_u = uint32(index.iman_u-1)';
index.mex.iumeasp_out = uint32(index.iumeasp_out-1)';
index.mex.ismeasp_out = uint32(index.ismeasp_out-1)';
index.mex.iq_out = uint32(index.iq_out-1)';
index.mex.isq_out = uint32(index.isq_out-1)';

%Create QP Matrices
[QP,state_est] = jMPC.create_QP(Model,pred.Phiq,Np,Nb,uwt,ywt,con,Kest,sizes,opts); 

%Perform System Scaling if Requested
if(opts.ScaleSystem)
    Hfac = opts.ScaleFac/max(max(abs(QP.H(1:sizes.Nbm_in,1:sizes.Nbm_in))));
    QP.Hscale = [Hfac*ones(sizes.Nbm_in,1);ones(con.soft.no,1)];  
    QP.H = diag(QP.Hscale)*QP.H;
    QP.R = chol(QP.H);
    if(~con.uncon)
        Afac = opts.ScaleFac/max(max(abs(con.A))); %dropping abs seems to help here?
        QP.Ascale = Afac*ones(size(con.bcon));
%         QP.Ascale = 1./max(abs(con.A),[],2); %seems to cause numerical issues?        
        con.A = diag(QP.Ascale)*con.A;
    else
        QP.Ascale = [];
    end
    QP.dualInc = 0.15;
else
    QP.Hscale = ones(sizes.Nb_in+con.soft.no,1);
    QP.Ascale = ones(size(con.bcon));
    %Find max element in H, A, b (unused at present)
    pmax = max([max(max(QP.H)) max(max(con.A)) max(con.bcon)]);
    QP.dualInc = max([0.15*pmax 0.15]);
end       
%Add Solver Choice to QP structure (for MEX)
if(~isempty(strfind(opts.QPSolver,'wright')))
    QP.solver = 1;
elseif(~isempty(strfind(opts.QPSolver,'mehrotra')))
    QP.solver = 2;
else
    QP.solver = 0;
end
%Add User Weights to QP structure
QP.uwt = uwt;
QP.ywt = ywt;

%Create sparse data structures for clp/ooqp
if(any(strcmpi(opts.QPSolver,{'clp','ooqp'})))
    QP.spH = sparse(tril(QP.H));
    con.spA = sparse(con.A);
end

%Default Initialisation  
initial.ym = Model.C*Model.x0;
initial.xm = x0;
initial.del_xm = Model.x0;
initial.u = init_u(index.iman_u)-lin.u_op(index.iman_u);
initial.up = zeros(n_in,1);
initial.up(index.iman_u) = init_u(index.iman_u)-lin.u_op(index.iman_u); %may need checking

%Build Return Structure
MPCstruct.Np = Np; 
MPCstruct.Nc = Nc; 
MPCstruct.Nb = Nb;
MPCstruct.Model = Model;           
MPCstruct.sizes = sizes;
MPCstruct.constraints = con;
MPCstruct.QP = QP;
MPCstruct.state_est = state_est; 
MPCstruct.pred = pred;
MPCstruct.lin = lin;
MPCstruct.initial = initial;
MPCstruct.index = index;
MPCstruct.opts = opts;
end