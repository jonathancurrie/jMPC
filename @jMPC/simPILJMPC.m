function simresult = simPILJMPC(J,simopts)
%MATLAB PIL MPC Simulation Engine
%
%   Called By jMPC/sim

%   Jonathan Currie (C)
%   AUT University 2012

%Allocate initial values
sPlant = simopts.Plant;
xp = simopts.initial.xp;
if(isfield(simopts.opts,'serialdevice'))
    s = simopts.opts.serialdevice;
else
    error('You must supply a serial port object via ''serialdevice'' to jSIM');
end

%Check we have a serial port
if(~isa(s,'serial'))
    error('You must supply a serial port for PIL simulations!');
end

%Variables
nm_in = J.sizes.nm_in;
n_out = J.sizes.n_out; 
states = J.sizes.states;
plotvec = simopts.plotvec;
T = zeros(simopts.T+1,1,'uint32');
iter = zeros(simopts.T+1,1,'uint16');
status = zeros(simopts.T+1,1,'int16');
if(J.mpcopts.Single)
    dtype = 'float';
    yp = single(simopts.initial.yp);
    mdist = single(simopts.mdist);
    setp = single(simopts.setp);
else
    dtype = 'double';
    yp = double(simopts.initial.yp);
    mdist = double(simopts.mdist);
    setp = double(simopts.setp);
end
if(J.mpcopts.WaitBar)
    h = waitbar(0,'Running jMPC PIL Simulation');
    kbar = 0;
end

%Open Serial Port
if(strcmp(s.Status,'closed'))
    fopen(s);
end
try
    %Run Simulation
    for k = 1:simopts.T
        %Send yp, mdist, setp to uC
        jMPC_SerialTx(s,yp);
        jMPC_SerialTx(s,mdist(k,:));
        jMPC_SerialTx(s,setp(k,:));

%         pause(0.01);
        %MPC Calculation done via processor in the loop

        %Collect results
        up = jMPC_SerialRx(s,nm_in,dtype);
        del_u = jMPC_SerialRx(s,nm_in,dtype);
        xm = jMPC_SerialRx(s,states,dtype);
        ym = jMPC_SerialRx(s,n_out,dtype);
        T(k+1) = jMPC_SerialRx(s,1,'uint32');
        iter(k+1) = jMPC_SerialRx(s,1,'uint16');
        status(k+1) = jMPC_SerialRx(s,1,'int16');

        %Update plant input disturbance
        usave = up;
        up = up + simopts.udist(k,:)';
        %Subject Plant To Input
        [xp,yp] = sim(sPlant,xp,double(up),simopts.Ts); %Ts used in nonlinear version
        %Update plant output disturbance
        if(J.mpcopts.Single)
            yp = single(yp + simopts.ydist(k,:)');
        else
            yp = yp + simopts.ydist(k,:)';
        end

        %Save plot vectors
        plotvec.xm(k+1,:) = xm;
        plotvec.xp(k+1,:) = xp;
        plotvec.del_u(k+1,:) = del_u;
        plotvec.u(k+1,:) = usave;
        plotvec.yp(k+1,:) = yp;
        plotvec.ym(k+1,:) = ym; %output augmented in states
        if(J.mpcopts.WaitBar)
            know = (k+1)/simopts.T;
            if(know-kbar >= 0.1)
                kbar = know;
                waitbar(kbar,h);
            end
        end
    end
catch ME
    if(J.mpcopts.WaitBar)
        close(h);
    end
    rethrow(ME);
end
%Force waitbar completition
if(J.mpcopts.WaitBar), waitbar(1,h); end   

%Create Result Object
simresult = simopts;
simresult.plotvec = plotvec;
simresult.qpstats.iter = iter;
simresult.qpstats.status = status;
simresult.opts.result = 1;
simresult.mode = 'PIL';
%Subtract Timing & Update
simresult.timing = struct('total',double(T)/1e6);
%Close waitbar and redraw
if(J.mpcopts.WaitBar), close(h); pause(0.00001); end
%Close serial port
fclose(s);
end