function simresult = simNLJMPC(J,simopts,verb)
%Non Linear MPC Simulation Engine
%
%   Called By jMPC/sim

%   Jonathan Currie (C)
%   AUT University 2011

%Get Bias Terms
u_op = J.mpcopts.lin.u_op;
y_op = J.mpcopts.lin.y_op;

%Allocate initial values
sModel = J.Model;
sPlant = simopts.Plant;
u = J.initial.u;
del_xm = J.initial.del_xm;
xm = J.initial.xm;
xp = simopts.initial.xp;
yp = simopts.initial.yp;

%Get index vectors
iman_u = J.index.iman_u;
imeas_dist = J.index.imeas_dist;
iumeasp_out = J.index.iumeasp_out;
ismeasp_out = J.index.ismeasp_out;

%Check for single precision simulation
if(J.mpcopts.Single)
    vtype = 'single';
else
    vtype = 'double';
end

%Variables
nm_in = J.sizes.nm_in;
n_out = J.sizes.n_out; 
nm_dist = J.sizes.nm_dist;
states = J.sizes.states;
v = zeros(1,J.sizes.nm_dist,vtype);
um = zeros(J.sizes.n_in,1,vtype); up = um;
K = zeros(length(del_xm),1,vtype);
plotvec = simopts.plotvec;
timing = struct('total',zeros(simopts.T+1,1),'state_est',zeros(simopts.T+1,1),...
                'rhs',zeros(simopts.T+1,1),'global',zeros(simopts.T+1,1),'qp',zeros(simopts.T+1,1),'del_u',zeros(simopts.T+1,1));
qpstats = struct('iter',zeros(simopts.T+1,1),'status',zeros(simopts.T+1,1));
h = waitbar(0,'Running jMPC Simulation');
try
    %Run Simulation
    for k = 1:simopts.T
        tstart = tic;
        %Re-Linearize Model & Re-build MPC
        if(k>2)
            J = J.NLupdate(u,xm);
            sModel = J.Model;
            u_op = J.mpcopts.lin.u_op;
            y_op = J.mpcopts.lin.y_op;
        end    
        %State Estimator Update
        K(ismeasp_out) = J.state_est.Kest*(yp-y_op);
        K(iumeasp_out) = del_xm(iumeasp_out); %Add model outputs for unmeasured outputs    
        del_xm = J.state_est.IKC*del_xm + K;    
        timing.state_est(k+1) = toc(tstart);
        %Calculate Input
        del_v = simopts.mdist(k,:);
        [del_u,stats] = mpcsolve(J,del_xm,u,simopts.setp,del_v,k,tstart); 
        %Select Optimal Input (just first sample)    
        optdel_u = del_u(1:nm_in);
        %Saturate Inputs
        [optdel_u,u] = J.saturate_inputs(optdel_u,u,nm_in,J.constraints);
        timing.del_u(k+1) = toc(tstart);
        %Concatenate measured disturbances
        if(nm_dist > 0)
            um(iman_u) = optdel_u;
            um(imeas_dist) = del_v';
            v = v + del_v; %sum measured disturbances (differenced)
            up(iman_u) = u;
            up(imeas_dist) = v';
        else
            um = optdel_u; %model input
            up = u;
        end

        %Simulate model
        del_xm = sim(sModel,del_xm,um); timing.total(k) = toc(tstart);
        %Sum State Increment
        xm = xm+del_xm(1:states); 

        %Update plant input disturbance & linerized bias
        usave = up;
        up = up + simopts.udist(k,:)' + u_op;
        %Subject Plant To Input
        [xp,yp] = sim(sPlant,xp,up,simopts.Ts); %Ts used in nonlinear version
        %Update plant output disturbance
        yp = yp + simopts.ydist(k,:)';

        %Save plot vectors & statistics
        plotvec.xm(k+1,:) = xm;
        plotvec.xp(k+1,:) = xp;
        plotvec.del_u(k+1,:) = optdel_u;
        plotvec.u(k+1,:) = usave;
        plotvec.yp(k+1,:) = yp;
        plotvec.ym(k+1,:) = del_xm(end-n_out+1:end); %output augmented in states
        plotvec.all_u(k+1,:) = del_u;
        plotvec.all_delx(k+1,:) = del_xm;
        timing.rhs(k+1) = stats.trhs;
        timing.global(k+1) = stats.tglobal;
        timing.qp(k+1) = stats.tqp;
        qpstats.iter(k+1) = stats.qpiter;
        qpstats.status(k+1) = stats.status;
        waitbar(k/simopts.T,h);
    end
catch ME
    close(h);
    rethrow(ME);
end

%Create Result Object
simresult = simopts;
simresult.plotvec = plotvec;
simresult.qpstats = qpstats;
simresult.timing = timing;
simresult.opts.result = 1;
if(J.mpcopts.Single)
    simresult.mode = 'Matlab (NLMPC) [Single Precision]';
else
    simresult.mode = 'Matlab (NLMPC) [Double Precision]';
end
close(h);
end