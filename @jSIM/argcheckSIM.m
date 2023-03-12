function argcheckSIM(Plant,sizes,T,setp,udist,mdist,ydist,opts)
%Detect Incorrect User Inputs for creating a jSIM Object
%
%   Called By jSIM Constructor

%   Jonathan Currie (C)
%   AUT University 2011

%Check obvious points
if(isempty(Plant))
    error('No Plant Supplied')
end

%Check time
if(T < 0)
    error('Simulation Time must be a positive number');
end

%Collect sizes
n_out = sizes.nq_out;
nm_out = sizes.nm_out;
n_in = sizes.n_in;
nm_dist = sizes.nm_dist;

%Check for nonlinear plant
if(~isa(Plant,'jNL'))
    %Check Plant x0
    states = size(Plant.B,1);
    if(length(Plant.x0) ~= states)
        error('Incorrect Number of Plant Initial Conditions (Plant.x0) - Expected %1d x %1d',states,1)
    end    
end

%Check correct number of plant outputs
if(~isa(Plant.C,'function_handle'))
    if(nm_out ~= size(Plant.C,1))
        error('Incorrect Number of Plant Outputs based on Measured Outputs Selected - Expected %1d',nm_out);
    end
end

%Check setpoint
if(size(setp,2) ~= n_out)
    if(sizes.nq_out ~= sizes.n_out)
        error('There are only %1d Controlled Outputs but %1d Setpoint Columns Provided - Expected %1d Columns',sizes.nq_out,sizes.n_out,sizes.nq_out);
    else
        error('Incorrect Number of Setpoints - Expected %1d Columns',n_out);
    end
end

%Check Disturbances
if(~isempty(udist))
    [r,c] = size(udist);
    if((r < T) || (c ~= n_in))
        error('Incorrect Number of Unmeasured Input (umdist) Disturbances - Expected %1d x %1d',T,n_in);
    end
end
if(~isempty(ydist))
    [r,c] = size(ydist);
    if((r < T) || (c ~= nm_out))
        error('Incorrect Number of Output (noise) Disturbances - Expected %1d x %1d',T,nm_out);
    end
end
if(~isempty(mdist))
    [r,c] = size(mdist);
    if((r < T) || (c ~= nm_dist))
        error('Incorrect Number of Measured Input (mdist) Disturbances - Expected %1d x %1d',T,nm_dist);
    end
end
end
