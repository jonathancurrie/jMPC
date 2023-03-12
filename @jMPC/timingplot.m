function h = timingplot(MPCobj,simres,mode)
%Plot MPC Simulation Results with Iteration & Timing Information
%
%   Called By jMPC\plot

%   Jonathan Currie (C)
%   Control Engineering 2011

%Distribute structure components
yp = simres.plotvec.yp;
u = simres.plotvec.u;
t = simres.timing.total*1000;
iter = simres.qpstats.iter;

%Create sample vector
k = 0:simres.T;
%Handle Vector
h = zeros(4,1);

%Setup Main Figure Name
figure(1);
set(gcf,'Name','jMPC Simulation - Timing','NumberTitle','off');
clf(gcf)

%****************** PLOT RESPONSE ******************%
%All in one Figure Overview

if(~MPCobj.constraints.uncon)
    if(MPCobj.constraints.ucon)
        con.u = MPCobj.constraints.u+MPCobj.StdModel.u_op(MPCobj.index.iman_u)*[1 1 0];
    end
    if(MPCobj.constraints.ycon)
        con.y = MPCobj.constraints.y+MPCobj.StdModel.y_op*[1 1];
    end
end

% Plot Output & Setpoint
subplot(411);
if(MPCobj.sizes.n_out == 1)
    plot(k,yp,'r');
else
    plot(k,yp);
end
h(1) = gca;
hold on; 
stairs(k,simres.setp(1:simres.T+1,:),'--','color',0.5*[1 1 1]); 
if(MPCobj.constraints.ycon) 
    if(MPCobj.sizes.n_out == 1)
        pm = 'r:';
    else
        pm = ':';
    end
    plot(xlim,[con.y(:,1)';con.y(:,1)'],pm);
    plot(xlim,[con.y(:,2)';con.y(:,2)'],pm);
end
myp = min(min(yp)); mxyp = max(max(yp));
ylim([myp-abs(myp/10) mxyp+mxyp/10]);
hold off;    
try
    set(gca,'XTickla','');
catch
end
title('Plant Output: yp(k)'); ylabel('Amplitude');

% Plot Input
subplot(412);
stairs(k,u);
h(2) = gca;
if(MPCobj.constraints.ucon)
    hold on;
    plot(xlim,[con.u(:,1)'; con.u(:,1)'],':');
    plot(xlim,[con.u(:,2)'; con.u(:,2)'],':');
    hold off;
end
mu = min(min(u)); mxu = max(max(u));
ylim([mu-abs(mu/10) mxu+mxu/10]);
try
    set(gca,'XTickla','');
catch
end
title('Input: u(k)'); ylabel('Amplitude');

%Plot Computation Time
subplot(413);
h(3) = gca;
if(~isempty(t))
    if(max(t) < 0.3)
        t = t*1e3; 
        lab = 'Time (\mus)'; tunit = '\mus'; funit = 'kHz';
    else
        lab = 'Time (ms)'; tunit = 'ms'; funit = 'Hz';
    end
    stairs(k,t,'m'); 
    title(sprintf('Computation Time [Average %1.2f%s, Max %1.2f%s (%1.2f%s)]',mean(t),tunit,max(t),tunit,1/max(t)*1e3,funit)); ylabel(lab);
    try
        set(gca,'XTickla',''); ylim([min(t)-min(t)/10 max(t)+max(t)/10]);
    catch
    end
end

% Plot Iterations
subplot(414);
h(4) = gca;
if(~isempty(iter))
    stairs(k,iter,'color',[0.7 0.3 0.2]);
    yl = [0 max(iter)+1]; ylim(yl);
    if(max(yl) >= MPCobj.mpcopts.QPMaxIter)
        hold on; plot(xlim,[MPCobj.mpcopts.QPMaxIter MPCobj.mpcopts.QPMaxIter],'r:'); hold off;
        if(max(yl) == MPCobj.mpcopts.QPMaxIter)
            ylim([yl(1) yl(2)+1]);
        end
    end
    title(sprintf('QP Iterations [Total %d, Max %d]',sum(iter),max(iter))); ylabel('# Iterations'); 
    xlabel('Sample (k)');
end

if(isfield(simres.timing,'rhs') && mode) %Timing detail not available from MEX
    %Create Timing Breakdown Figure
    figure(2);
    set(gcf,'Name','jMPC Simulation - Timing Breakdown','NumberTitle','off');
    clf(gcf)

    %Calculate Differences
    t = simres.timing;
    model = t.total-t.del_u;
    del_u = t.del_u-t.qp;
    qp = t.qp-t.global;
    glob = t.global-t.rhs;
    rhs = t.rhs-t.state_est;
    state_est = t.state_est;

    %Group & Plot
    y = [state_est rhs glob del_u model qp]*1000;
    bar(y,'stack')
    axis([0 simres.T ylim])
    legend('State Estimation','RHS Update','Global Minimum Check','Delta U Update','Model Simulation','QP Solve')
    if(~mode); title('MPC Timing Breakdown'); end;
    xlabel('Sample (k)'); ylabel('Time (ms)');
end