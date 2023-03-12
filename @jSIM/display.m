function displaySIM(simopt)
%Display Simulation Information
%
%   Called By jSIM Class

%   Jonathan Currie (C)
%   Control Engineering 2011

%**************** Title ****************%
disp('------------------------------------------------');
if(~simopt.opts.result); disp('MPC SIMULATION OPTIONS');
else disp('MPC SIMULATION RESULTS');
end
disp('------------------------------------------------');
%Simulation
disp('  Simulation Specifications:')
if(simopt.opts.result)
    fprintf('Simulation Mode:        %s\n',simopt.mode);
end
if(isa(simopt.Plant,'jNL'))
    switch(simopt.Plant.odesolver)
        case 0;  fprintf('Plant Type:             ODE (ode45)\n');
        case 1;  fprintf('Plant Type:             ODE (ode23s)\n');
        case 2;  fprintf('Plant Type:             ODE (ode15s)\n');    
    end    
else
    fprintf('Plant Type:             Linear State Space\n');
end
fprintf('Length Of Simulation:   %1.2f sec\n',simopt.simulink.Tfinal);
% fprintf('Sampling Time:          %1.2f sec\n',simopt.Ts);
disp(' ');
if(~simopt.opts.result)
    if(max(max(simopt.ydist)) ~= 0)
        disp('Output Disturbances:    YES')
    else
        disp('Output Disturbances:    NO')
    end
    if(max(max(simopt.udist)) ~= 0)
        disp('Input Disturbances:     YES')
    else
        disp('Input Disturbances:     NO')
    end
else   
    if(~isempty(simopt.timing.total))
         avgT = mean(simopt.timing.total); maxT = max(simopt.timing.total);
        if(maxT < 0.01)
            avgT = avgT*1e3; maxT = maxT*1e3; tunit = 'ms'; funit = 'kHz';
        else
            tunit = 's'; funit = 'Hz';
        end
        fprintf('Sampling Statistics:\n');        
        fprintf('- Average:  %1.3f %s (%1.3f %s)\n',avgT,tunit,1/avgT,funit);
        fprintf('- Maximum:  %1.3f %s (%1.3f %s)\n',maxT,tunit,1/maxT,funit);   
    end
    if(~isempty(simopt.qpstats.iter))
        fprintf('Solver Statistics:\n');
        fprintf('- Total QP Iters: %d (Max %d)\n',sum(simopt.qpstats.iter),max(simopt.qpstats.iter));
        fprintf('- Total QP Failures: %d\n',sum(simopt.qpstats.status < 0));
    end
end
disp('------------------------------------------------');
%**************** Warnings ****************%
% if(~isempty(simopt.warning))
%     disp('  WARNINGS:')    
%     for i = 1:length(simopt.warning)
%         disp(simopt.warning{i})
%     end
%     disp('------------------------------------------------');
% end

end