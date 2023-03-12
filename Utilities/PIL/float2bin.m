function [out] = float2bin(in)
%Return the binary equivalent of a decimal number in floating point format
%assuming single precision, 23 bit fraction

out = dec2bin(hex2dec(num2hex(single(in))),32);

end

