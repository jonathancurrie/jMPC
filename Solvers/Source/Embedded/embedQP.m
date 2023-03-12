function [bytes,QPres] = embedQP(H,f,A,b,maxiter,tol,solver,precision,verbosity)
%Generate Embedded QP Solver

if(nargin < 9 || isempty(verbosity)), verbosity = 0; end
if(nargin < 8 || isempty(precision)), precision = 'single'; end
if(nargin < 7 || isempty(solver)), solver = 'mehrotra'; end
if(nargin < 6 || isempty(tol)), tol = 1e-4; end
if(nargin < 5 || isempty(maxiter)), maxiter = 100; end

global strprec prec constbytes dynbytes verb
constbytes = 0; dynbytes = 0; verb = verbosity;

%Check Sensible Tolerance
switch(precision)
    case 'single'
        if(tol < 1e-6), error('jMPC cannot generate a single precision implementation with a QPTol <= 1e-6'); end
        precision = 'float'; %modify for use internally        
    case 'double'
        if(tol < 1e-9), error('jMPC cannot generate a double precision implementation with a QPTol <= 1e-9'); end                
end
prec = precision;
%Modify strprecision based on precision we are generating
if(strcmpi(precision,'double'))
    strprec = '%1.18g';
else
    strprec = '%1.10g';
end

%------------------------------------------------------------------------%
%Command Window Display
if(verb)
    fprintf('\n------------------------------------------------\n');
    fprintf('Auto Code Generator for Embedded QP\n');
    fprintf('Data Type: %s    Precision: %s\n',prec,strprec);
    fprintf('------------------------------------------------\n');
end

%------------------------------------------------------------------------%
%Stage 1 Create Embedded Header File
if(verb), fprintf('\nStage 1:\nCreating Embedded Header File "jMPC_embed.h" ... '); end
%Open & Write Header
fid = fopen_embed('jMPC_embed.h');
fprintf(fid,'#define MLMODE 1 //QP Testing Mode\n');
switch(lower(solver))
    case 'wright'
        fprintf(fid,'#define QPSolver 0 //QuadWright\n');
    case 'mehrotra'
        fprintf(fid,'#define QPSolver 1 //QuadMehrotra\n');
end
%Copy from Header Template File
copytemplate(fid,'jMPCEmbed.h',0);
%Close File
fprintf(fid,'\n');
if(verb), fprintf('Done\n'); end
fclose(fid);

%------------------------------------------------------------------------%
%Stage 2 Create Embedded Constants File
if(verb), fprintf('\nStage 2:\nCreating Embedded Constants File "jMPC_constants.c" ... '); end
%Open & Write Header
fid = fopen_embed('jMPC_constants.c');
%QP Constants
ndec = length(f); mc = length(b);
fprintf(fid,'// QP CONSTANTS\n');
write_carr(fid,H','H');
write_carr(fid,A','A');
write_cvar(fid,tol,'tol');
write_cvar(fid,maxiter,'maxiter','unsigned int');
write_cvar(fid,ndec,'ndec','unsigned int');
write_cvar(fid,mc,'mc','unsigned int');
if(any(strcmpi(prec,{'double','long double'})))
    write_cvar(fid,eps(1),'MACH_EPS');
else
    write_cvar(fid,eps(single(1)),'MACH_EPS');
end
%QP Vars (cheating really)
fprintf(fid,'\n\n// QP Vars\n');
write_arr(fid,f','f');
write_arr(fid,b','b');
write_arr(fid,zeros(ndec,1),'x');
write_arr(fid,zeros(mc,1),'lambda');
write_arr(fid,zeros(mc,1),'tslack');

%Close File
fprintf(fid,'\n');
if(verb), fprintf('Done\n'); end
fclose(fid); 

%------------------------------------------------------------------------%
% STAGE 3 - Create jMath Source File
if(verb), fprintf('\nStage 3:\nCreating jMATH Source File "jMPC_math.c" ... '); end
fid = fopen_embed('jMPC_math.c');
%Add new include
fprintf(fid,'#include "jMPC_embed.h"\r\n');
%Copy from Template File
copytemplate(fid,'jmath_embed.c');

%------------------------------------------------------------------------%
% STAGE 4 - Create QP Source File
if(verb), fprintf('\nStage 4:\nCreating QP Source File "jMPC_qp.c"'); end
fid = fopen_embed('jMPC_qp.c');
%Add new includes
fprintf(fid,'#include "jMPC_embed.h"\n');
fprintf(fid,'#include <math.h>\n');
%Add Req Global Vars
fprintf(fid,'// Global Variables //\n'); 
write_arr(fid,zeros(mc,1),'del_lam');
write_arr(fid,zeros(mc,1),'del_t');
write_arr(fid,zeros(ndec,1),'del_z');
write_arr(fid,zeros(mc,1),'ilamt');
write_arr(fid,zeros(mc,1),'lamt');
write_arr(fid,zeros(ndec,1),'r1');
write_arr(fid,zeros(mc,1),'r2');
write_arr(fid,zeros(mc,1),'mesil');
if(strcmpi(solver,'mehrotra'))
    write_arr(fid,zeros(ndec,1),'rhs');
    write_arr(fid,zeros(mc,1),'igr2');
end
write_arr(fid,zeros(ndec,ndec),'RQP');
%Copy from Template File
switch(lower(solver))
    case 'mehrotra'
        if(verb), fprintf(' [Mehrotra] ... '); end
        copytemplate(fid,'qpmehrotra_embed.c');
    case 'wright'
        if(verb), fprintf(' [Wright] ... '); end
        copytemplate(fid,'qpwright_embed.c');
    otherwise
        error('unknown solver');
end

fid = fopen_embed('jMPC_engine.c'); fclose(fid);

%------------------------------------------------------------------------%
% STAGE 6 - Create PIL Source File
if(verb), fprintf('\nStage 6:\nCreating PIL MPC Source File "jMPC_pil.c" ... '); end
fid = fopen_embed('jMPC_pil.c');
%Add new includes
fprintf(fid,'#include "jMPC_embed.h"\n');
%Add precision definition
if(any(strcmp(prec,{'double','long double'})))
    fprintf(fid,'#define DOUBLE_PREC\n');
end
%Add Req Global Vars
% fprintf(fid,'// Global Variables //\n');
% write_arr(fid,0,'yp');
% write_arr(fid,0,'mdist');   
% write_arr(fid,0,'setp');    
% write_arr(fid,0,'mpc_u');
% write_arr(fid,0,'mpc_del_u');    
% write_arr(fid,0,'mpc_xm');    
% write_arr(fid,0,'mpc_ym'); 

%Copy from Template File
copytemplate(fid,'serial_embed.c');

%------------------------------------------------------------------------%
%Add local vars
dynbytes = dynbytes + 19*getsize(prec) + 10*getsize('int');
%Create return structures
bytes.total = constbytes+dynbytes;
bytes.flash = constbytes;
bytes.ram = dynbytes;
if(verb)
    fprintf('\n------------------------------------------------\n');
    fprintf('Auto Generated Embedded Code Summary:\n') 
    fprintf('TOTAL: %6.3f KB\n',(constbytes+dynbytes)/1000);
    fprintf('FLASH: %6.3f KB\n',constbytes/1000); 
    fprintf('RAM:   %6.3f KB\n',dynbytes/1000);
    fprintf('------------------------------------------------\n\n');
end




function fid = fopen_embed(str)
%Open new autogen file for writing, printing header as we go

%Open File
fid = fopen(str,'wt'); 
%Check we got a valid identifier
if(fid == -1)
    error('Could not open file: %s',str);
end
%Otherwise print header
fprintf(fid,'/*------------------------------------------------\n');
fprintf(fid,'  jMPC Auto Code Generator for Embedded QP\n'); 
fprintf(fid,'  Jonathan Currie (www.controlengineering.co.nz)\n'); 
fprintf(fid,'  Code Generated on: %s\n',datestr(now));
fprintf(fid,'  ------------------------------------------------ */\n\n');

function copytemplate(fid,template,doClose,QPMehrotra)
%Copy contents of template file to autogen file
global prec arch verb

try
    str = fileread(template);
catch ME
    fclose(fid);
    error('Error reading: %s\nError: %s',template,ME.message);
end
if(nargin < 4), QPMehrotra = 0; end
if(nargin < 3), doClose = 1; end

%Find //EMBED START, remove it
ind = strfind(str,'//EMBED START');
if(isempty(ind))
    fclose(fid);
    error('Could not find //EMBED START'); 
end
str(1:ind+13) = [];

%Remove extra line returns (matlab adds them)
ind = strfind(str,13); str(ind) = []; 

%Check and replace if required precision
if(strcmp(prec,'float'))
    str = regexprep(str,'double','float'); 
    str = regexprep(str,'0\.0','0.0F');
    str = regexprep(str,'1\.0','1.0F');
    str = regexprep(str,'2\.0','2.0F');
    str = regexprep(str,'0\.15','0.15F');
    str = regexprep(str,'0\.5','0.5F');
    str = regexprep(str,'0\.1 ','0.1F ');
    str = regexprep(str,'0\.1,','0.1F,'); 
    str = regexprep(str,'0\.1)','0.1F)'); 
    str = regexprep(str,'0\.1;','0.1F;'); 
    str = regexprep(str,'0\.999995','0.999995F');
    str = regexprep(str,'0\.99999 ','0.99999F ');
    str = regexprep(str,'0\.9995','0.9995F');    
%     str = regexprep(str,'sqrt','sqrtf'); %ccs doesn't know about sqrtf
elseif(any(strcmp(prec,{'double','long double'})) && strcmpi(arch,'C2000')) %TI C2000 float=double
    str = regexprep(str,'double','long double');
end
if(QPMehrotra)
    str = regexprep(str,'qpwright','qpmehrotra');
end
%Write to autogen file
fprintf(fid,'%s\n\n',str);
%Close autogen file
if(doClose)
    fclose(fid);
    if(verb), fprintf('Done\n'); end
end

function write_carr(fid,var,name,dtype)
%Write MATLAB variable to C file as a constant array
global prec
if(nargin < 4), dtype = prec; end
writeVar(fid,var,name,'const',dtype,1);

function write_arr(fid,var,name,dtype)
%Write MATLAB variable to C file as an array
global prec
if(nargin < 4), dtype = prec; end
writeVar(fid,var,name,'',dtype,1);

function write_cvar(fid,var,name,dtype)
%Write MATLAB variable to C file as constant variable
global prec
if(nargin < 4), dtype = prec; end
writeVar(fid,var,name,'const',dtype,0);

function write_var(fid,var,name,dtype)
%Write MATLAB variable to C file as normal variable
global prec
if(nargin < 4), dtype = prec; end
writeVar(fid,var,name,'',dtype,0);


function writeVar(fid,var,name,constness,dtype,isptr)
%Write MATLAB variable to C file, adding bytes as we go
global strprec constbytes dynbytes

%Determine if writing floating point or integer
switch(dtype)
    case {'long double','double','float'}
        strp = strprec;
    otherwise
        strp = '%d';
end

%Determine size and #bytes of variable
[r,c] = size(var); mul = getsize(dtype);

%Add constness
if(~isempty(constness)), dtype = sprintf('%s %s',constness,dtype); end
%Update Byte Counter
switch(constness)
    case 'const'
        constbytes = constbytes + r*c*mul;
    otherwise
        dynbytes = dynbytes + r*c*mul;
end
    
%Ensure we have numbers (not logicals)
if(islogical(var)), var = uint32(var); end    

%Replace Infs with BIG numbers (not used anyway)
ind = isinf(var); sv = sign(var) > 0;
if(any(ind))
    var(ind & sv) = 1e38;
    var(ind & ~sv) = -1e38;
end

if(isempty(var))
    if(isptr)
        fprintf(fid,'%s %s[1] = {0};\n',dtype,name);
    else
        fprintf(fid,'%s %s = 0;\n',dtype,name);
    end
elseif(r==1 && c==1) %scalar
    if(isptr)
        fprintf(fid,['%s %s[1] = {' strp '};\n'],dtype,name,var);
    else
        fprintf(fid,['%s %s = ' strp ';\n'],dtype,name,var);
    end
elseif(r==1) %row vector
    fprintf(fid,'%s %s [%d] = {',dtype,name,c);
    for i = 1:c-1
        fprintf(fid,[strp ','],var(i));
    end
    fprintf(fid,[strp '};\n'],var(i+1));
elseif(c==1) %column vector
    fprintf(fid,'%s %s[%d] = {',dtype,name,r);
    for i = 1:r-1
        fprintf(fid,[strp ','],var(i));
    end
    fprintf(fid,[strp '};\n'],var(i+1));    
else %matrix
    fprintf(fid,'%s %s[%d] = {\n',dtype,name,r*c); 
    for i = 1:r-1
        fprintf(fid,['\t' strp ','],var(i,1));
        for j = 2:c-1
            fprintf(fid,[strp ','],var(i,j));
        end
        if(isempty(j)), j = c-1; end;
        fprintf(fid,[strp ',\n'],var(i,j+1)); 
    end
    fprintf(fid,['\t' strp ','],var(i+1,1));
    for j = 2:c-1
        fprintf(fid,[strp ','],var(i+1,j));
    end
    if(isempty(j)), j = c-1; end;
    fprintf(fid,[strp '};\n'],var(i+1,j+1));
end

function mul = getsize(type)
%Calculate increase in size
switch(type)
    case {'long double','double'}; mul = 8;
    case {'long long','unsigned long long'}; mul = 8;
    case {'long','unsigned long','long int','unsigned long int'}; mul = 4;
    case {'single','float'}; mul = 4;
    case {'int','unsigned int'}; mul = 2;
    case {'short','unsigned short','short int','unsigned short int'}; mul = 2;
    case {'char','unsigned char'}; mul = 2;
    otherwise; mul = 4; %average guess
end