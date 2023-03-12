function jMPCtool()
%Launch the jMPC Simulation Tool

%   Jonathan Currie (C)
%   Control Engineering 2009 

try
    jMPC_GUI();
catch ME
    rethrow(ME)
end

end