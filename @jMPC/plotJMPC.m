function h = plotJMPC(MPCobj,simres,mode,plot_model)
%Plot MPC Simulation Results
%
%   Called By jMPC\plot

%   Jonathan Currie (C)
%   Control Engineering 2009

%Distribute structure components
u = simres.plotvec.u; 
yp = simres.plotvec.yp;  
xp = simres.plotvec.xp;
xm = simres.plotvec.xm; 
del_u = simres.plotvec.del_u; 

%Collect sizes
n_in = MPCobj.sizes.n_in;
n_out = MPCobj.sizes.n_out;
nm_out = MPCobj.sizes.nm_out;
nm_dist = MPCobj.sizes.nm_dist;
%Collect index vectors
iumeas_out = MPCobj.index.iumeas_out;
iq_out = MPCobj.index.iq_out;
iman_u = MPCobj.index.iman_u;
imeas_dist = MPCobj.index.imeas_dist;
 
if(strcmp(simres.mode,'MPC Toolbox'))
    ym = NaN;
else
    ym = simres.plotvec.ym; 
end

%Create sample vector
k = 0:simres.T;

%Set Defaults
col = 2;        %two columns
porder = [1 2 3 4]; %plot in normal order

%Check for disturbances
if(simres.opts.dist); 
    col = 3;
    porder = [1 2 4 5 3 6];
end

%Check for labels
if(~isempty(MPCobj.mpcopts.InputNames)); inNlabel = 1; else inNlabel = 0; end
if(~isempty(MPCobj.mpcopts.InputUnits)); inUlabel = 1; else inUlabel = 0; end
if(~isempty(MPCobj.mpcopts.OutputNames)); outNlabel = 1; else outNlabel = 0; end
if(~isempty(MPCobj.mpcopts.OutputUnits)); outUlabel = 1; else outUlabel = 0; end

%Check for NL sim
if(isa(MPCobj.StdModel,'jNL'))
    Model = MPCobj.StdModel.lin_model;
else
    Model = MPCobj.StdModel;
end

%Setup Main Figure Name
h(1) = figure(1);
set(gcf,'NumberTitle','off');
switch(lower(simres.mode))
    case 'matlab'
        set(gcf,'Name','jMPC Matlab Simulation');
    case 'simulink'
        set(gcf,'Name','jMPC Simulink Simulation');
    case 'mex'
        set(gcf,'Name','jMPC MEX Simulation');
    case 'mpc toolbox'
        set(gcf,'Name','MPC Toolbox Simulation');
    case 'pil'
        set(gcf,'Name','jMPC PIL Implementation');
end
clf(gcf)

%****************** PLOT RESPONSE ******************%
%All in one Figure Overview

% Plot Output & Setpoint
subplot(2,col,porder(1));
if(plot_model) 
    plot(k,yp,k,ym,'*--'); %only plot ym if enabled
elseif(n_out ~= nm_out)
    plot(k,yp,k,ym(:,iumeas_out),'-.');
else
    plot(k,yp);
end
hold on; stairs(k,simres.setp(1:simres.T+1,:),'k--'); hold off;
yl = ylim;
axis([0 simres.T yl(1)+yl(1)/10, yl(2)+yl(2)/10]);
if(plot_model) 
    title('Outputs: y_p(k) & y_m(k)');    
elseif(n_out ~= nm_out)
    title('Outputs: y_p(k) & Unmeasured Outputs y_m(k)');
else
    title('Outputs: y_p(k)'); 
end
ylabel('Amplitude');


% Plot States
subplot(2,col,porder(2));
if(plot_model) 
    plot(k,xp,k,xm,'*--'); %only plot xm if enabled
else
    plot(k,xp);
end
if(plot_model) 
    title('States: x_p(k) & x_m(k)');    
else
    title('States: x_p(k)'); 
end
axis([0 simres.T ylim]);
ylabel('Amplitude');

 % Plot Input
subplot(2,col,porder(3));
if(nm_dist > 0)    
    [ta,ua] = stairs(k,u(:,iman_u));
    [tb,ub] = stairs(k,u(:,imeas_dist));
    plot(ta,ua,tb,ub,'-.');
    title('Input: u(k) & Measured Disturbances: v(k)'); xlabel('Sample'); ylabel('Amplitude');
else
    stairs(k,u)
    title('Input: u(k)'); xlabel('Sample'); ylabel('Amplitude');
end
axis([0 simres.T ylim]);

%Plot Change in Input
subplot(2,col,porder(4));
stairs(k,del_u)
title('Change in Input: \Delta u(k)'); xlabel('Sample'); ylabel('Amplitude');
axis([0 simres.T ylim]);

if(simres.opts.dist)
    % Plot Output Disturbance
    subplot(2,col,porder(5));
    plot(k,simres.ydist)
    axis([0 simres.T ylim]);
    title('Measurement Noise'); ylabel('Amplitude');

    %Plot Input Disturbance
    subplot(2,col,porder(6));
    stairs(k,simres.udist(1:simres.T+1,:))
    axis([0 simres.T ylim]);
    title('Unmeasured Input Disturbance'); xlabel('Sample'); ylabel('Amplitude');
end

%****************** PLOT RESPONSE MODE 2 ******************%
% Separate figures for inputs and outputs   
if(MPCobj.constraints.delucon)
    con.delu = MPCobj.constraints.u(:,3);
end
if(MPCobj.constraints.ucon)
    con.u = MPCobj.constraints.u+Model.u_op(iman_u)*[1 1 0];
end
if(MPCobj.constraints.ycon)
    con.y = MPCobj.constraints.y+Model.y_op*[1 1];
end
if(nm_dist > 0)
    v = simres.mdist;% + repmat(Model.u_op(imeas_dist)',size(simres.mdist,1),1);
end

if(strcmp(mode,'detail'))  
    %Create a new axis for each input
    h(2) = figure(2);
    clf
    set(gcf,'Name','Inputs','NumberTitle','off');
    rows = double(n_in);
    T = simres.T;
    iu = 1; id = 1;
    for i = 1:n_in
        plot1 = double(max(i*2-1,1));
        plot2 = double(max(i*2,2));
        %Plot Input
        subplot(rows,2,plot1);
        if(any(i == imeas_dist)) %Measured Disturbance
            stairs(k,u(1:T+1,imeas_dist(id)),'color',[0.2 0.6 0.6])
            if(inNlabel); title(['u_' num2str(i) ': ' MPCobj.mpcopts.InputNames{imeas_dist(id)} ' (MeasDist)']); else title(['u_',num2str(i)]); end
            if(inUlabel); ylabel(['[' MPCobj.mpcopts.InputUnits{imeas_dist(id)} ']']); else ylabel('Amplitude'); end        
            if(i == n_in)
                xlabel('Sample');
            end  
            axis([0 simres.T ylim]);
        else %Manipulated Input
            stairs(k,u(1:T+1,iman_u(iu)),'color',[0.2 0.6 0.2])
            if(MPCobj.constraints.ucon)
                maxin = max(u(:,iu));
                minin = min(u(:,iu));
                yl = ylim;
                if(con.u(iu,2)-maxin < maxin/thresh(con.u(iu,2))) %response is close to constraint           
                    hold on
                    plot([0 T],[con.u(iu,2) con.u(iu,2)],':k');
                    hold off
                    ymax = con.u(iu,2)+(con.u(iu,2)-yl(1))*0.1; %upper cons + 1/10 range of plot
                elseif(con.u(iu,2) < yl(2)) %constraint is already within ylimits
                    hold on
                    plot([0 T],[con.u(iu,2) con.u(iu,2)],':k');
                    hold off
                    ymax = yl(2);
                else %ignore constraint
                    ymax = yl(2);
                end
                if(abs(minin-con.u(iu,1)) < abs(minin/thresh(con.u(iu,1))))
                    hold on
                    plot([0 T],[con.u(iu,1) con.u(iu,1)],':k');
                    hold off
                    ymin = con.u(iu,1)-(ymax-con.u(iu,1))*0.1; %lower cons - 1/10 range of plot
                elseif(con.u(iu,1) > yl(1))
                    hold on
                    plot([0 T],[con.u(iu,1) con.u(iu,1)],':k');
                    hold off
                    ymin = yl(1);
                else
                    ymin = yl(1);
                end
                axis([0 simres.T ymin ymax]);
            else
                axis([0 simres.T ylim]);
            end
            if(inNlabel); title(['u_' num2str(i) ': ' MPCobj.mpcopts.InputNames{iman_u(iu)}]); else title(['u_',num2str(i)]); end
            if(inUlabel); ylabel(['[' MPCobj.mpcopts.InputUnits{iman_u(iu)} ']']); else ylabel('Amplitude'); end        
            if(i == n_in)
                xlabel('Sample');
            end
        end

        %Plot Change in Input
        subplot(rows,2,plot2);
        if(any(i == imeas_dist)) %Measured Disturbance
            stairs(k,v(1:T+1,id),'m')
            if(inNlabel); title(['\Deltau_' num2str(i) ': Change in ' MPCobj.mpcopts.InputNames{imeas_dist(id)} ' (MeasDist)']); else title(['\Deltau_',num2str(i)]); end      
            if(i == n_in)
                xlabel('Sample');
            end            
            id = id + 1;
            axis([0 simres.T ylim]);
        else %Manipulated Input
            stairs(k,del_u(:,iu),'r')
            if(MPCobj.constraints.delucon)            
                maxDin = max(del_u(:,iu));
                minDin = min(del_u(:,iu));
                if(con.delu(iu)-maxDin < maxDin/10)
                    hold on                        
                    plot([0 T],[con.delu(iu) con.delu(iu)],':k');
                    hold off
                    ymax = con.delu(iu)*1.1;
                else
                    yl = ylim; ymax = yl(2);
                end
                if(abs(minDin--con.delu(iu)) < abs(maxDin/10))
                    hold on                        
                    plot([0 T],[-con.delu(iu) -con.delu(iu)],':k');
                    hold off
                    ymin = -con.delu(iu)*1.1;
                else
                    yl = ylim; ymin = yl(1);
                end
                axis([0 simres.T ymin ymax]);
            else
                axis([0 simres.T ylim]);
            end
            if(inNlabel); title(['\Deltau_' num2str(i) ': Change in ' MPCobj.mpcopts.InputNames{iman_u(iu)}]); else title(['\Deltau_',num2str(i)]); end
            if(i == n_in)
                xlabel('Sample');
            end
            iu = iu + 1;
        end
    end
    %Create a new axis for each output
    h(3) = figure(3);
    clf
    if(n_out~=nm_out)
        set(gcf,'Name','Outputs & Unmeasured Outputs','NumberTitle','off');
    else
        set(gcf,'Name','Outputs','NumberTitle','off');
    end
    rows = double(n_out); ip = 1; im = 1; 
    for i = 1:n_out
        %Plot Output & Setpoint
        subplot(rows,1,double(i));        
        if(n_out ~= nm_out)
            if(plot_model)
                if(any(i == iumeas_out))
                    plot(k,ym(:,i),'*--');
                else
                    plot(k,yp(:,ip),k,ym(:,i),'*--'); ip = ip + 1;
                end
            else
                if(any(i == iumeas_out))
                    plot(k,ym(:,i),'-.');
                else
                    plot(k,yp(:,ip)); ip = ip + 1;
                end
            end
        elseif(plot_model)
            plot(k,yp(:,i),k,ym(:,i),'*--');
        else
            plot(k,yp(:,i));
        end
        hold on; 
        if(any(iq_out-i == 0))
            stairs(k,simres.setp(1:simres.T+1,im),'k--'); im = im + 1;
        end
        hold off
        if(MPCobj.constraints.ycon)
            if(i>nm_out)    
                maxin = max(ym(:,i));
                minin = min(ym(:,i));
            else
                maxin = max(yp(:,i));
                minin = min(yp(:,i));
            end
            yl = ylim;
            if(con.y(i,2)-maxin < maxin/thresh(con.y(i,2))) %response is close to constraint           
                hold on
                plot([0 T],[con.y(i,2) con.y(i,2)],':k');
                hold off
                ymax = con.y(i,2)+(con.y(i,2)-yl(1))*0.1;
            elseif(con.y(i,2) < yl(2)) %constraint is already within ylimits
                hold on
                plot([0 T],[con.y(i,2) con.y(i,2)],':k');
                hold off
                ymax = yl(2);
            else %ignore constraint
                ymax = yl(2);
            end
            if(abs(minin-con.y(i,1)) < abs(minin/thresh(con.y(i,1))))
                hold on
                plot([0 T],[con.y(i,1) con.y(i,1)],':k');
                hold off
                ymin = con.y(i,1)-(ymax-con.y(i,1))*0.1;
            elseif(con.y(i,1) > yl(1))
                hold on
                plot([0 T],[con.y(i,1) con.y(i,1)],':k');
                hold off
                ymin = yl(1);
            else
                ymin = yl(1);
            end  
            axis([0 simres.T ymin ymax]);
        end         
        if(outNlabel)
            if(any(i == iumeas_out))
                title(['y_' num2str(i) ': ' MPCobj.mpcopts.OutputNames{i} ' (Unmeasured)']); 
            else
                title(['y_' num2str(i) ': ' MPCobj.mpcopts.OutputNames{i}]); 
            end
        else
            title(['y_',num2str(i)]);
        end
        if(outUlabel); ylabel(['[' MPCobj.mpcopts.OutputUnits{i} ']']); else ylabel('Amplitude'); end
        if(i == n_out)
            xlabel('Sample');
        end
    end
end
end


function y = thresh(x)
    %Calculate threshold factor for determining whether response is close
    %to constraint
    %
    %   i.e. if resp = 1.8, threshold = cons/1
    %        if resp = 18, threshold = cons/10
    %        if resp = 0.18, threshold = cons/10
    
        
    if(x == 0)
        y = 1e6; %assume large enough
    else
        y = 10^floor(log10(abs(x)));
        if(y < 1) %correct for small numbers
            y = 1/y;
        end
    end
end
        
 
 

