function data = jMPC_SerialRx(s,no,type)
% Receive Data via Serial Port from HIL jMPC

if(nargin < 3)
    type = 'float';
end

% Read data
data = fread(s,double(no),type);