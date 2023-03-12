function simopts = buildSIM(MPCobj,Plant,T,setp,udist,mdist,ydist)
%Build Simulation Options
%
%   Called By jSIM Constructor

%   Jonathan Currie (C)
%   AUT University 2011

sizes = MPCobj.sizes;
index = MPCobj.index;
lin = MPCobj.lin;
opts = MPCobj.mpcopts;

%Warnings
warn_count = 1;
simopts.warning = [];

%Subtract Operating Point if Linearized Model is supplied
if(lin.islin)
    setp = setp-repmat(lin.y_op_full(index.iq_out)',size(setp,1),1);     
end

%******************** Common Construction *********************%
%Setpoint
len = size(setp,1);
if(len ~= (T+1)) %shorter than expected (expects T+1 but will cope with T)
    if(len == 1) %auto expand setpoint vector
        setpexp = zeros(T+1,sizes.n_out);
        for i = 1:sizes.n_out
            setpexp(:,i) = setp(1,i)*ones(T+1,1);
        end
        setp = setpexp;
    elseif(len == T)
        extra = T+1-len;
        setpexp = zeros(extra,sizes.nq_out);
        for i = 1:extra
            setpexp(i,:) = setp(end,:);
        end
        setp = [setp;setpexp];           
    else 
        simopts.warning{warn_count} = sprintf('Setpoint vector is too short, last row is being copied - Expected %1d x %1d',T,sizes.n_out);
        extra = T+1-len;
        setpexp = zeros(extra,sizes.n_out);
        for i = 1:extra
            setpexp(i,:) = setp(end,:);
        end
        setp = [setp;setpexp];
    end
end
%Expand setpoint vector by Np if look ahead required
if(opts.LookAhead)
    setpexp = ones(MPCobj.Np,sizes.nq_out);
    for i = 1:sizes.nq_out
        setpexp(:,i) = setpexp(:,i)*setp(end,i);
    end
    setp = [setp;setpexp];
end

%Flag for plotting
if(isempty(udist) && isempty(ydist))
    simopts.opts.dist = 0;
else
    simopts.opts.dist = 1;
end

%Initial Plant Output & States
if(isa(Plant.C,'function_handle'))
    simopts.initial.yp = Plant.C([],Plant.x0,MPCobj.initial.u,Plant.param);
else
    simopts.initial.yp = Plant.C*Plant.x0;
end
simopts.initial.xp = Plant.x0;

%U Dist
len = size(udist,1);
if(isempty(udist))
    udist = zeros(T+1,sizes.n_in);
elseif(len == T)
    extra = T+1-len;
    udistexp = zeros(extra,sizes.n_in);
    for i = 1:extra
        udistexp(i,:) = udist(end,:);
    end
    udist = [udist;udistexp];
end
%Y Dist
len = size(ydist,1);
if(isempty(ydist))
    ydist = zeros(T+1,sizes.nm_out);
elseif(len == T)
    extra = T+1-len;
    ydistexp = zeros(extra,sizes.nm_out);
    for i = 1:extra
        ydistexp(i,:) = ydist(end,:);
    end
    ydist = [ydist;ydistexp];
end
%Meas Dist
if(~isempty(mdist))  
    len = size(mdist,1);
    if(len == T)
        extra = T+1-len;
        mdistexp = zeros(extra,sizes.nm_dist);
        for i = 1:extra
            mdistexp(i,:) = mdist(end,:);
        end
        mdist = [mdist;mdistexp];
    end
    %Save Simulink Mdist (not differenced and do not remove lin point)
    sim_mdist = mdist;
    %Subtract Mdist operating point (not supplied as delta)
    mdist = mdist - repmat(lin.u_op(index.imeas_dist)',size(mdist,1),1);    
    %Difference Mdist
    mdist(2:T+1,:) = mdist(2:T+1,:)-mdist(1:T,:);
    mdist(T+1,:) = zeros(1,sizes.nm_dist);
else
    mdist = zeros(T+1,max(sizes.nm_dist,1));
    sim_mdist = mdist;
end



%Preallocate Plot vectors
plotvec.yp = zeros(T+1,length(simopts.initial.yp));
plotvec.xp = zeros(T+1,length(Plant.x0));
plotvec.del_u = zeros(T+1,sizes.nm_in);
plotvec.u = zeros(T+1,sizes.n_in);
plotvec.ym = zeros(T+1,sizes.n_out);
plotvec.xm = zeros(T+1,sizes.states);

%Initialise vectors with sample 0 data
plotvec.xp(1,:) = Plant.x0;
plotvec.xm(1,:) = MPCobj.initial.xm(1:sizes.states)+MPCobj.Model.x0(1:sizes.states);
plotvec.u(1,MPCobj.index.iman_u) = MPCobj.initial.u;
plotvec.yp(1,:) = simopts.initial.yp;
plotvec.ym(1,:) = MPCobj.initial.ym;

%Save into Class property
simopts.plotvec = plotvec;

%******************** Simulink Construction *********************%
Ts = MPCobj.Model.Ts;
%Construct a setpoint vector
simopts.simulink.setp.time = 0:Ts:T*Ts;     
simopts.simulink.setp.signals.values = setp;
%Construct a udist vector
simopts.simulink.udist.time = 0:Ts:T*Ts;     
simopts.simulink.udist.signals.values = udist;
%Construct a ydist vector (out by one sample compared to Matlab sim)
simopts.simulink.ydist.time = 0:Ts:(T+1)*Ts;     
simopts.simulink.ydist.signals.values = [zeros(1,sizes.nm_out); ydist];
%Construct a mdist vector
simopts.simulink.mdist.time = 0:Ts:T*Ts;     
simopts.simulink.mdist.signals.values = sim_mdist;
%Create Final Time
simopts.simulink.Tfinal = Ts*T;
%Create Mdist Index
[~,simopts.simulink.mdist_ind] = sort([index.imeas_dist index.iman_u]);

%******************** Allocate Variables *********************%
simopts.Ts = Ts;
simopts.setp = setp;
simopts.udist = udist;
simopts.ydist = ydist;
simopts.mdist = mdist;
simopts.opts.result = 0;
end