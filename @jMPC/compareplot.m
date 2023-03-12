function [dy,du,h] = compareplot(sim1,sim2,plotmode)
%Compare two MPC Simulation Studies
%
%   Called By jMPC\compare

%   Jonathan Currie (C)
%   AUT University 2011

%Distribute structure components
yp1 = sim1.plotvec.yp;
u1 = sim1.plotvec.u;
yp2 = sim2.plotvec.yp;
u2 = sim2.plotvec.u;

%Check we have iter/stat data for plotmode > 1
if(plotmode > 1 && (isempty(sim2.qpstats.iter) || isempty(sim1.qpstats.iter)))
    plotmode = 1;
end

%Determine Lengths
if(size(yp1,1) > size(yp2,1))
    len = size(yp2,1);
else
    len = size(yp1,1);
end

%Calculate 'errors'
ey = yp1(1:len,:)-yp2(1:len,:);
eu = u1(1:len,:)-u2(1:len,:);
dy = norm(ey); 
du = norm(eu); 
h = [];

if(plotmode)
    %Detailed Comparison
    if(plotmode > 1)
        pr = 2; pc = 4; pi = 2; h = zeros(8,1);
    else
        pr = 2; pc = 2; pi = 0; h = zeros(4,1);
    end
    
    %Create sample vectors
    k1 = 0:sim1.T;
    k2 = 0:sim2.T;

    %Setup Main Figure Name
    figure(1);
    set(gcf,'Name','jMPC Simulation - Comparison Plot','NumberTitle','off');
    clf(gcf)

    %****************** PLOT COMPARISON ******************%
    % Plot Responses
    if(~isfield(sim1,'setp')), ks = k2; setp = sim2.setp; else ks = k1; setp = sim1.setp; end
    subplot(pr,pc,1); h(1) = gca;
    plot(k1,yp1,k2,yp2); hold on; stairs(ks,setp,'--','color',0.5*[1 1 1]); hold off;
    title('Plant Output: yp(k)'); ylabel('Amplitude');
    set(gca,'XTicklabel','');
    axis([0 min(sim1.T,sim2.T) ylim]);

    subplot(pr,pc,2); h(2) = gca;
    [t,us] = stairs(k1,u1);
    [t1,us1] = stairs(k2,u2);
    plot(t,us,t1,us1);
    title('Input: u(k)'); 
    set(gca,'XTicklabel','');
    axis([0 min(sim1.T,sim2.T) ylim]);

    % Plot Differences
    if(size(yp1,1) > size(yp2,1))
        len = size(yp2,1);
    else
        len = size(yp1,1);
    end
    k = 0:len-1;

    subplot(pr,pc,3+pi); h(3) = gca;
    plot(k,ey); hold on; plot([0 k(end)],[0 0],':','color',0.35*[1 1 1]); hold off;
    title('Output Difference'); ylabel('y1-y2'); xlabel('Sample (k)');
    %Determine Limits
    mey = max(max(abs(ey))); mey = mey + mey/10;
    if(mey ~= 0)
        axis([0 min(sim1.T,sim2.T) -mey mey ]);
    else
        axis([0 min(sim1.T,sim2.T) ylim ]);
    end
    
    subplot(pr,pc,4+pi); h(4) = gca;
    plot(k,eu); hold on; plot([0 k(end)],[0 0],':','color',0.35*[1 1 1]); hold off;
    title('Input Difference'); ylabel('u1-u2'); xlabel('Sample (k)');
    meu = max(max(abs(eu))); meu = meu + meu/10;
    if(meu ~= 0)
        axis([0 min(sim1.T,sim2.T) -meu meu]);
    else
        axis([0 min(sim1.T,sim2.T) ylim]);
    end
    
    if(plotmode > 1)
        s1iter = double(sim1.qpstats.iter); s1stat = double(sim1.qpstats.status);
        s2iter = double(sim2.qpstats.iter); s2stat = double(sim2.qpstats.status);
        
        subplot(pr,pc,3); h(5) = gca;
        [t,is] = stairs(k1,s1iter);
        [t1,is1] = stairs(k2,s2iter);
        plot(t,is,t1,is1);
        axis([0 min(sim1.T,sim2.T) ylim]);
        title('QP Iterations'); ylabel('Iters'); 
        
        subplot(pr,pc,3+4); h(6) = gca;
        stairs(k,s1iter-s2iter);
        axis([0 min(sim1.T,sim2.T) ylim]);
        title('QP Iterations Difference'); ylabel('i1-i2'); xlabel('Sample (k)');
        
        subplot(pr,pc,4); h(7) = gca;
        bar(k,[s1stat s2stat]);
%         axis([0 min(sim1.T,sim2.T) ylim]);
        title('QP Status'); ylabel('Exit Code'); 
        
        subplot(pr,pc,4+4); h(8) = gca;
        stairs(k,s1stat-s2stat);
        axis([0 min(sim1.T,sim2.T) ylim]);
        title('QP Status Difference'); ylabel('e1-e2'); xlabel('Sample (k)');
    end
end

