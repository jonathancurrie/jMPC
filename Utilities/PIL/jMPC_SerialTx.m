function jMPC_SerialTx(s,data)
% Send Data via Serial Port to HIL jMPC

%Write Header
fwrite(s,202);
%Write No
fwrite(s,length(data));
%Write Data
fwrite(s,data,class(data));