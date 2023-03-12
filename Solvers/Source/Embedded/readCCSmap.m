function bytes = readCCSmap(filename)
%READCCSMAP  Read CCS .map file and return RAML2 and RAML3 memory count

bytes = struct('flash',0,'ram',0,'initram',0,'code',0,'table',0,'total',0);
d = fileread(filename);
l0 = textscan(d(strfind(d,'RAML0'):end),'%s',5); bytes.table = hex2dec(l0{1}{4})*2;
l1 = textscan(d(strfind(d,'RAML1'):end),'%s',5); bytes.code = hex2dec(l1{1}{4})*2;
l2 = textscan(d(strfind(d,'RAML2'):end),'%s',5); bytes.ram = hex2dec(l2{1}{4})*2;
l3 = textscan(d(strfind(d,'RAML3'):end),'%s',5); bytes.flash = hex2dec(l3{1}{4})*2;
h0 = textscan(d(strfind(d,'RAMH0'):end),'%s',5); bytes.initram = hex2dec(h0{1}{4})*2;
bytes.total = bytes.flash+bytes.ram+bytes.table+bytes.code+bytes.initram;