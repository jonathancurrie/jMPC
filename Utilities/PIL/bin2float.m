function out = bin2float(in)
% Converts a float represented in binary to its decimal equivalent

% 01000000000000000000000000000000; //fp 2
% 01000000010000000000000000000000; //fp 3

if(length(in) ~= 32)
    error('Only Accepts 32bit Floating Point Binary');
end

B=zeros(24,1);
B(1) = 1;
j = 2;

%Convert fraction to logic form
for i=10:32
    if strcmp(in(i),'1')
        B(j) = 1; 
    else 
        B(j) = 0;
    end 
    j = j+1;
end

k = single(bin2dec(in(2:9))-127);

if(k < 0)
    B = [zeros(abs(k),1);B];
    k = 0;
end

y = single(0);

for i = 1:23
    y = y + 2^k*B(i);
    k = k - 1;
end

if strcmp(in(1),'1') 
    y = -y;
end

out = y;

end