function [bytes,QPres,MPCres,h] = embedJMPC(J,simopts,opts)
%Embed MPC Controller
%
%   Called By jMPC/embed

%   Copyright (C) Jonathan Currie
%   Industrial Information & Control Centre (I2C2) 2012-2013

global strprec prec constbytes dynbytes qpbytes mpcbytes tol maxiter arch verb
constbytes = 0; dynbytes = 0; qpbytes = 0; mpcbytes = 0; 
embedver = 1.5; h = [];
verb = opts.verbose;

%Check for precision match
if(strcmpi(opts.precision,'match'))
    if(J.mpcopts.Single)
        opts.precision = 'single';
    else
        opts.precision = 'double';
    end
end
%Check Single/Double
if(J.mpcopts.Single && strcmpi(opts.precision,'double'))
    error('jMPC cannot generate a double precision implementation from a single precision controller');
end
%Check Ref Testbench Precision
if(opts.verifympc && J.mpcopts.Single && strcmpi(opts.tbprecision,'double'))
    error('jMPC cannot convert a single precision MPC controller to double precision for testbench verification. You must start with a double precision controller.'); 
end
%Check Solver
if(J.QP.solver == 0)
    error('jMPC can only generate controllers with quad_wright or quad_mehrotra solvers (not %s)',J.mpcopts.QPSolver);
end
%Check Sensible Tolerance
switch(opts.precision)
    case 'single'
        if(J.mpcopts.QPTol < 1e-6), error('jMPC cannot generate a single precision implementation with a QPTol <= 1e-6'); end
        opts.precision = 'float'; %modify for use internally        
    case 'double'
        if(J.mpcopts.QPTol < 1e-9), error('jMPC cannot generate a double precision implementation with a QPTol <= 1e-9'); end                
end
%Modify strprecision based on precision we are generating
if(isempty(opts.strprecision))
    if(strcmpi(opts.precision,'double'))
        strprec = '%1.18g';
    else
        strprec = '%1.10g';
    end
else
    strprec = opts.strprecision;
end
%Check for NL plant
if(~isempty(simopts) && isa(simopts.Plant,'jNL') && opts.verifympc)
    jmpcwarn('jMPC:Embed','Cannot Verify generated MPC Controller with a Nonlinear Plant Model - Skipping');
    opts.verifympc = 0;
end

%Setup globals
prec = opts.precision;
tol = J.mpcopts.QPTol;
maxiter = J.mpcopts.QPMaxIter;
arch = opts.arch;

%Modify TI C2000 double to long double
if(strcmpi(arch,'c2000') && strcmpi(prec,'double'))
    prec = 'long double';
end

%Modify QP solver for best chance of validation
QPMehrotra = J.QP.solver==2;
switch(prec)
    case {'double','long double'}
        if(J.QP.solver == 1) %wright
            J.mpcopts.QPSolver = 'mquad_wright';
        else
            J.mpcopts.QPSolver = 'mquad_mehrotra';
        end      
    otherwise
        if(J.QP.solver == 1) %wright
            J.mpcopts.QPSolver = 'msquad_wright';
        else
            J.mpcopts.QPSolver = 'msquad_mehrotra';
        end 
end

%------------------------------------------------------------------------%
%Command Window Display
if(verb)
    fprintf('------------------------------------------------\n');
    fprintf('jMPC Auto Code Generator for Embedded MPC [v%1.1f]\n',embedver);
    fprintf('Architecture: %s,  Precision: %s [%s]\n',arch,prec,strprec);
    fprintf('------------------------------------------------\n');
end

%Output dir
copytb = false;
if(isempty(opts.dir))
    odir = [cd '\'];
else
    odir = opts.dir;
    if(odir(end) ~= '\')
        odir = [odir '\'];
    end
    copytb = true;
end

%------------------------------------------------------------------------%
%Stage 1 Create Embedded Header File
if(verb), fprintf('1)  Creating Embedded Header File "jMPC_embed.h" ... '); end
%Open & Write Header
fid = fopen_embed([odir 'jMPC_embed.h'],embedver);

%If benchmarking, assume we have standard libraries for memcpy etc
if(opts.genbench)
    fprintf(fid,'//Common Headers\n');
    fprintf(fid,'#include <string.h>\n');
end

%Add Type Definitions
fprintf(fid,'\n//Type Definitions\n');
fprintf(fid,'typedef %s drealT;\n','double');
fprintf(fid,'typedef %s realT;\n',prec);
fprintf(fid,'typedef %s intT;\n','int');
fprintf(fid,'typedef %s uintT;\n','unsigned int');
fprintf(fid,'typedef %s longT;\n','long');
fprintf(fid,'typedef %s ulongT;\n','unsigned long');
fprintf(fid,'typedef %s ullongT;\n','unsigned long long');
fprintf(fid,'typedef %s charT;\n','char');
fprintf(fid,'typedef %s ucharT;\n','unsigned char');

%Software Defines
fprintf(fid,'\n//Software Defines\n');
fprintf(fid,'#define JMPCVER "%s"\n',sprintf('%1.2f',jMPCver));
fprintf(fid,'#define EMBEDVER "%s"\n',sprintf('%1.2f',embedver));
idx = strfind(J.mpcopts.QPSolver,'quad');
fprintf(fid,'#define QPSOLVER "%s"\n',upper(J.mpcopts.QPSolver(idx:end)));
switch(prec)
    case {'double','long double'}
        fprintf(fid,'#define PRECISION "DOUBLE"\n');     
    otherwise
        fprintf(fid,'#define PRECISION "SINGLE"\n');      
end

%Add Problem Specific Defines
fprintf(fid,'\n//QP Configuration Defines\n');
fprintf(fid,'#define ACCELQP %d  //Accelerate QP Solver\n',opts.accelqp);

fprintf(fid,'\n//MPC Configuration Defines\n');
if(~J.constraints.uncon),  fprintf(fid,'#define CON_MPC    //Constrained MPC\n'); end
if(J.constraints.delucon), fprintf(fid,'#define DELU_CON   //Delta U Constraints Exist\n'); end
if(J.constraints.ucon),    fprintf(fid,'#define U_CON      //Input Constraints Exist\n'); end
if(J.constraints.ycon),    fprintf(fid,'#define Y_CON      //Output Constraints Exist\n'); end
if(J.sizes.nm_dist > 0),   fprintf(fid,'#define MEAS_DIST  //Measured Disturbances Present\n'); end
if(J.sizes.nq_out ~= J.sizes.n_out), fprintf(fid,'#define UNCON_OUT  //Uncontrolled Outputs Present\n'); end
if(J.sizes.nm_out ~= J.sizes.n_out), fprintf(fid,'#define UNMEAS_OUT //Unmeasured Outputs Present\n'); end

fprintf(fid,'\n//Actual Implementation Configuration Defines\n');
fprintf(fid,'#define MPC_INTERVAL %d  //Sampling Interval in us\n',J.Model.Ts*1e6);

%Copy from Header Template File
fprintf(fid,'\n');
copytemplate(fid,'jMPCEmbed.h',0);
%Close File
fprintf(fid,'\n');
if(verb), fprintf('Done\n'); end
fclose(fid);

%------------------------------------------------------------------------%
%Stage 2 Create Embedded Constants File
if(verb), fprintf('2)  Creating Embedded Constants File "jMPC_constants.c" ... '); end
%Open & Write Header
fid = fopen_embed([odir 'jMPC_constants.c'],embedver);
%Add new includes
fprintf(fid,'#include "jMPC_embed.h"\n\n');
%QP Constants
fprintf(fid,'// QP CONSTANTS\n');
writeQPConstants(fid,J,opts.accelqp);
%MPC Constants
fprintf(fid,'\n// MPC CONSTANTS\n');
writeMPCConstants(fid,J);
%Close File
fprintf(fid,'\n');
if(verb), fprintf('Done\n'); end
fclose(fid); 

%------------------------------------------------------------------------%
% STAGE 3 - Create jMath Source File
if(verb), fprintf('3)  Creating jMATH Source File "jMPC_math.c" ... '); end
fid = fopen_embed([odir 'jMPC_math.c'],embedver);
%Add new include
fprintf(fid,'#include "jMPC_embed.h"\r\n');
%Copy from Template File
copytemplate(fid,'jMathEmbed.c');

%Only include QP stuff if constrained
if(~J.constraints.uncon)
    %------------------------------------------------------------------------%
    % STAGE 4 - Create QP Source File
    if(verb), fprintf('4)  Creating QP Source File "jMPC_qp.c"'); end
    fid = fopen_embed([odir 'jMPC_qp.c'],embedver);
    %Add new includes
    fprintf(fid,'#include "jMPC_embed.h"\n');
    fprintf(fid,'#include <math.h>\n');
    %Add Req Global Vars
    writeQPGlobals(fid,J,QPMehrotra,true);
    %Copy from Template File
    if(QPMehrotra)
        if(verb), fprintf(' [Mehrotra] ... '); end
        copytemplate(fid,'QPMehrotraEmbed.c',0);
    else
        if(verb), fprintf(' [Wright] ... '); end
        copytemplate(fid,'QPWrightEmbed.c',0);
    end
    %Copy Utilities
    copytemplate(fid,'QPUtils.c');

    %Create Testbench if we have simopts
    if(~isempty(simopts))
        %------------------------------------------------------------------------%
        % STAGE 4a - Create QP Testbench
        if(opts.verifyqp)
            if(verb), fprintf('4a) Creating MEX QP Testbench Source File "mexTestQP.c" ... '); end
            fid = fopen_embed([odir 'mexTestQP.c'],embedver);
            %Add new includes
            fprintf(fid,'#include "mex.h"\n');
            fprintf(fid,'#include "math.h"\n');
            fprintf(fid,'#include "jMPC_embed.h"\n');
            %Add Defines
            switch(prec)
                case {'float'}
                    fprintf(fid,'#define MPC_CLASS %s\n\n','mxSINGLE_CLASS');
                case {'double' 'long double'}
                    fprintf(fid,'#define MPC_CLASS %s\n\n','mxDOUBLE_CLASS');
                otherwise
                    fclose(fid);
                    error('Unknown prec: %s',prec);
            end
            %Add Req Global Vars
            writeQPTestGlobals(fid,J,simopts);
            %Copy from Template File
            copytemplate(fid,'TestQP.c',1,QPMehrotra);
        end
    end
end

%------------------------------------------------------------------------%
% STAGE 5 - Create MPC Source File
if(verb), fprintf('5)  Creating MPC Source File "jMPC_engine.c" ... '); end
fid = fopen_embed([odir 'jMPC_engine.c'],embedver);
%Add new includes
fprintf(fid,'#include "jMPC_embed.h"\n');
%Add Req Global Vars
writeMPCGlobals(fid,J);
%Copy from Template File
copytemplate(fid,'jMPCEmbed.c',1,QPMehrotra);

%Can't create testbench if no simopts
if(~isempty(simopts))
    %------------------------------------------------------------------------%
    % STAGE 5a - Create MPC Testbench
    if(opts.verifympc)
        if(verb), fprintf('5a) Creating MEX MPC Testbench Source File "mexTestMPC.c" ... '); end
        fid = fopen_embed([odir 'mexTestMPC.c'],embedver);
        %Add new includes
        fprintf(fid,'#include "mex.h"\n');
        fprintf(fid,'#include "jMPC_embed.h"\n');

        %Add Defines
        switch(prec)
            case {'float'}
                fprintf(fid,'#define MPC_CLASS %s\n\n','mxSINGLE_CLASS');
            case {'double' 'long double'}
                fprintf(fid,'#define MPC_CLASS %s\n\n','mxDOUBLE_CLASS');
            otherwise
                fclose(fid);
                error('Unknown prec: %s',prec);
        end

        %Add Req Global Vars
        writeMPCTestGlobals(fid,J,simopts,false);
        %Copy from Template File
        copytemplate(fid,'TestMPC.c');
    end
    
    %------------------------------------------------------------------------%
    % STAGE 5b - Create MPC Benchmark
    if(opts.genbench)
        if(verb), fprintf('5b) Creating MPC Benchmark Source File "benchMPC.c" ... '); end
        fid = fopen_embed([odir 'benchMPC.c'],embedver);
        %Add new includes
        fprintf(fid,'#include "jMPC_embed.h"\n');
        %Add timing headers
        switch(lower(opts.arch))
            case 'generic'
                fprintf(fid,'#include <Windows.h>\n');
            case 'arm' %assume linux
                fprintf(fid,'#include <time.h>\n');
        end
        fprintf(fid,'#include <stdio.h>\n');
        
        %Add Req Global Vars
        writeMPCTestGlobals(fid,J,simopts,true);
        %Copy from Template File
        copytemplate(fid,'benchmarkMPC.c');
    end

    %------------------------------------------------------------------------%
    % STAGE 6 - Create PIL Source File
    if(verb), fprintf('6)  Creating PIL MPC Source File "jMPC_pil.c" ... '); end
    fid = fopen_embed([odir 'jMPC_pil.c'],embedver);
    %Add new includes
    fprintf(fid,'#include "jMPC_embed.h"\n');
    %Add precision definition
    if(any(strcmp(prec,{'double','long double'})))
        fprintf(fid,'#define DOUBLE_PREC\n');
    end
    fprintf(fid,'#ifndef NO_PIL\n');
    %Add Req Global Vars
    writePILGlobals(fid,J,simopts);
    %Copy from Template File
    copytemplate(fid,'PILEmbed.c',1,QPMehrotra);
end

%------------------------------------------------------------------------%
%Add local vars
qpbytes = qpbytes + 18*getsize(prec) + 5*getsize('int');
mpcbytes = mpcbytes + 1*getsize(prec) + 5*getsize('int');
dynbytes = dynbytes + 19*getsize(prec) + 10*getsize('int');
%Create return structures
bytes.total = constbytes+dynbytes;
bytes.flash = constbytes;
bytes.ram = dynbytes;
bytes.qp = qpbytes;
bytes.mpc = mpcbytes;
if(verb)
    fprintf('------------------------------------------------\n');
    fprintf('Data Memory Summary:\n') 
    fprintf('TOTAL: %6.3f KB\n',(constbytes+dynbytes)/1000);
    fprintf('FLASH: %6.3f KB    QP:  %6.3f KB\n',constbytes/1000,qpbytes/1000); 
    fprintf('RAM:   %6.3f KB    MPC: %6.3f KB\n',dynbytes/1000,mpcbytes/1000);
    fprintf('------------------------------------------------\n');
end

%If verify selected, compile testbench files
if(opts.verifyqp || opts.verifympc)
    if(isempty(simopts))
        if(verb), jmpcwarn('jMPC:Embed','Without simulation options QP and MPC verification cannot be run'); end
        QPres = [];
        MPCres = [];
    else
        if(verb)
            fprintf('------------------------------------------------\n');
            fprintf('Auto Generated Embedded Code Verification:\n');
        end
        if(opts.verifyqp)
            if(J.constraints.uncon)
                if(verb), jmpcwarn('jMPC:Embed','Unconstrained MPC - No QP To Verify'); end
            else
                clear mexTestQP
                cstr = ['mex -largeArrayDims "' odir 'mexTestQP.c" "' odir 'jMPC_QP.c" "' odir 'jMPC_math.c" "' odir 'jMPC_constants.c"'];                
                if(verb), fprintf('Compiling MEX QP Testbench... '); end
                eval(cstr);
                if(verb), fprintf('Done\n'); end
                if(copytb)
                    try
                        copyfile([cd '\mexTestQP.' mexext],[odir 'mexTestQP.' mexext],'f');
                    catch
                    end
                end
            end
        end
        if(opts.verifympc)
            clear mexTestMPC
            if(verb), fprintf('Compiling MEX MPC Testbench... '); end
            cstr = ['mex -largeArrayDims "' odir 'mexTestMPC.c" "' odir 'jMPC_engine.c" "' odir 'jMPC_math.c" "' odir 'jMPC_constants.c"']; 
            if(~J.constraints.uncon)
                cstr = [cstr ' "' odir 'jMPC_QP.c"'];
            end
            eval(cstr);
            if(verb), fprintf('Done\n'); end
            if(copytb)
                try
                    copyfile([cd '\mexTestMPC.' mexext],[odir 'mexTestMPC.' mexext],'f');
                catch
                end
            end
        end
        fprintf('\n');
        %Generate pass/fail threshold
        switch(prec)
            case {'volatile float','float'}
                thres = 1e-4;
            case {'double' 'long double'}
                thres = 1e-6;
        end
        %Defaults
        QPres = []; MPCres = [];

        %Now Run Comparisons
        if(opts.verifyqp && ~J.constraints.uncon)
            [QPres.dx,QPres.status] = mexTestQP;
            %Print Pass Fail
            if(verb)
                if(QPres.dx > thres)
                    fprintf('MEX QP Verification FAILED\n');
                else
                    fprintf('MEX QP Verification PASSED\n');
                end
                switch(QPres.status)
                    case 1, fprintf('- Successfully Solved\n');
                    case -1, fprintf('- Maximum Iterations Exceeded\n');
                    case -2, fprintf('- Cholesky Failure\n');                            
                    case -3, fprintf('- Numerical Errors Encountered\n');
                    case -4, fprintf('- Infeasible\n');
                    otherwise, fprintf('- Unknown Exit Status\n');
                end
                fprintf('- z_norm: %1.5g\n',QPres.dx);
            end
            QPres.pass = QPres.dx <= thres;
            if(opts.verifympc), fprintf('\n'); end
        end
        if(opts.verifympc)
            sim1.plotvec = mexTestMPC;
            sim1.qpstats.iter = sim1.plotvec.qpiter;
            sim1.qpstats.status = sim1.plotvec.qpstat;
            sim1.T = simopts.T;
            %Determine if we need to convert the supplied controller to single precision
            if(strcmpi(opts.tbprecision,'double') || (strcmpi(opts.tbprecision,'match') && any(strcmpi(prec,{'double','long double'}))))
                Jsim = J;
            else
                Jsim = single(J);
            end
            Jsim.mpcopts.mexWarn = false;
            if(isa(simopts.Plant,'jNL'))
                sim2 = sim(Jsim,simopts,'Matlab');
            else
                sim2 = sim(Jsim,simopts,'Mex');
            end
            MPCres.plotvec = sim1.plotvec;
            MPCres.simres_tb = sim1;
            MPCres.simres_ref = sim2;
            [MPCres.dy,MPCres.du,h] = J.compareplot(sim1,sim2,opts.compareplot);
            %Plot Comparison if required
            if(opts.compareplot)            
                set(gcf,'Name','Embedded MPC Verification - MATLAB Results vs AutoCode Results');
            end
            %Print Pass Fail
            if(verb)
                if(MPCres.dy > thres)
                    fprintf('MEX MPC Verification FAILED on y_norm\n');
                elseif(MPCres.du > thres)
                    fprintf('MEX MPC Verification FAILED on u_norm\n');
                else
                    fprintf('MEX MPC Verification PASSED\n');
                end
                fprintf('- y_norm: %1.5g\n- u_norm: %1.5g  \n',MPCres.dy,MPCres.du);
            end
            MPCres.pass = MPCres.du <= thres && MPCres.dy <= thres;
        end
        if(verb), fprintf('------------------------------------------------\n\n'); end
    end
else
    QPres = [];
    MPCres = [];
end


function fid = fopen_embed(str,jver)
%Open new autogen file for writing, printing header as we go
global arch prec strprec
%Open File
fid = fopen(str,'wt'); 
%Check we got a valid identifier
if(fid == -1)
    error('Could not open file: %s',str);
end
%Otherwise print header
fprintf(fid,'/*------------------------------------------------\n');
fprintf(fid,'  jMPC Auto Code Generator for Embedded MPC\n'); 
fprintf(fid,'  Version %1.1f by Jonathan Currie (www.i2c2.aut.ac.nz)\n',jver); 
fprintf(fid,'  Code Generated on: %s\n  Architecture: %s, Precision: %s [%s]\n',datestr(now),arch,prec,strprec);
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

function writeQPGlobals(fid,J,QPMehrotra,doAdd)
%Write global QP variables to file
global addmpc addbytes
addmpc = false; %writing qp bytes
addbytes = doAdd;
mc = length(J.constraints.bcon);
ndec = length(J.QP.Hscale);

fprintf(fid,'// Global Variables //\n'); 
write_arr(fid,NaN(mc,1),'del_lam');
write_arr(fid,NaN(mc,1),'del_t');
write_arr(fid,NaN(ndec,1),'del_z');
write_arr(fid,NaN(mc,1),'ilamt');
write_arr(fid,NaN(mc,1),'lamt');
write_arr(fid,NaN(ndec,1),'r1');
write_arr(fid,NaN(mc,1),'r2');
write_arr(fid,NaN(mc,1),'mesil');
if(QPMehrotra)
    write_arr(fid,NaN(ndec,1),'rhs');
    write_arr(fid,NaN(mc,1),'igr2');
end
write_arr(fid,NaN(ndec,ndec),'RQP');


function writeQPConstants(fid,J,accelqp)
%Write global QP constants to file
global prec tol maxiter addmpc addbytes
addmpc = false; %writing qp bytes
addbytes = true;
ndec = length(J.QP.Hscale);
mc = length(J.constraints.bcon);

write_carr(fid,J.QP.H','H');
write_carr(fid,J.constraints.A','A');
if(accelqp > 0)
    write_carr(fid,J.constraints.A,'At');
    if(accelqp > 1)
        AtA = zeros(ndec,ndec,mc); % Preallocate J for n*n*m elements of storage
        for k = 1:mc
            AtA(:,:,k) = J.constraints.A(k,:)'*J.constraints.A(k,:);
        end
        AtA = reshape(AtA,ndec^2,mc);
        %Don't need all rows
        idx = find(tril(ones(ndec^2)));
        write_carr(fid,AtA,'AtA');        
        write_carr(fid,idx-1,'AtAidx','int');
    end
end
write_cvar(fid,tol,'tol');
write_cvar(fid,maxiter,'maxiter','int');
write_cvar(fid,ndec,'ndec','int');
write_cvar(fid,mc,'mc','int');
write_cvar(fid,max(max(abs([J.QP.H; J.constraints.A]))),'pHAmax');
if(any(strcmpi(prec,{'double','long double'})))
    write_cvar(fid,eps(1),'MACH_EPS');
else
    write_cvar(fid,eps(single(1)),'MACH_EPS');
end

function writeQPTestGlobals(fid,J,simopts)
%Write global QP Test constants to header filer
global addbytes
addbytes = false; %don't count test bytes
%Get initial conditions
k = 1;
u = J.initial.u;
del_xm = J.initial.del_xm;
yp = simopts.initial.yp;
K = zeros(length(del_xm),1);
%Get Bias Terms
y_op = J.lin.y_op;
%Get Indices
ismeasp_out = J.index.ismeasp_out;
iumeasp_out = J.index.iumeasp_out;
%Generate Initial b, f
K(ismeasp_out) = J.state_est.Kest*(yp-y_op);
K(iumeasp_out) = del_xm(iumeasp_out); %Add model outputs for unmeasured outputs    
del_xm = J.state_est.IKC*del_xm + K;    
del_v = simopts.mdist(k,:);    
[b,f] = update_rhs(J,del_xm,u,simopts.setp,del_v,k);
%Generate initial solution
z0 = zeros(J.sizes.Nb_in+J.constraints.soft.no,1);
%Calculate Reference Answer
[del_u,stats] = mpcsolve(J,del_xm,u,simopts.setp,del_v,k,tic,true);

fprintf(fid,'// Global Variables //\n'); 
write_carr(fid,b,'b');
write_carr(fid,f,'f');
write_arr(fid,z0,'z');
write_arr(fid,NaN(size(b)),'lambda');
write_arr(fid,NaN(size(b)),'tslack');
fprintf(fid,'// Reference Solution //\n');
write_carr(fid,del_u,'rdel_u');
write_cvar(fid,stats.qpiter,'riter','unsigned int');
write_cvar(fid,stats.status,'rstatus','int');


function writeMPCGlobals(fid,J)
%Write global MPC variables to file
global addbytes addmpc
addmpc = true; %writing mpc bytes
addbytes = true; %don't count test bytes
statesn = J.sizes.states+J.sizes.n_out;
statesnm = J.sizes.states+J.sizes.nm_out;
statesnq = J.sizes.states+J.sizes.nq_out;
Np_out = J.Np*J.sizes.n_out;
Np_qout = J.Np*J.sizes.nq_out;
mc = length(J.constraints.bcon);
ndec = length(J.QP.Hscale);

fprintf(fid,'//-- Global Variables --//\n');
write_arr(fid,zeros(statesnm,1),'Kys');
write_arr(fid,NaN(statesn,1),'K');
write_arr(fid,NaN(J.sizes.n_out,1),'ys');
write_arr(fid,zeros(Np_out,1),'y0');
if(J.sizes.nq_out ~= J.sizes.n_out)
    write_arr(fid,NaN(statesnq,1),'qp_del_xm');
end    
write_arr(fid,NaN(Np_qout,1),'Ssetp');
write_arr(fid,NaN(ndec,1),'x');
write_arr(fid,zeros(ndec,1),'fvec');
write_arr(fid,NaN(J.sizes.n_in,1),'um');
write_arr(fid,NaN(J.sizes.n_in,1),'up');
if(J.sizes.nm_dist > 0)
    write_arr(fid,zeros(J.sizes.nm_dist,1),'old_v'); %can't be NaN as we don't set initial v
    write_arr(fid,NaN(J.sizes.nm_dist,1),'del_v');
end
write_arr(fid,NaN(J.sizes.Nb_in,1),'del');
write_arr(fid,NaN(mc,1),'bvec');
write_arr(fid,NaN(mc,1),'bcheck');
%Loop vars
write_arr(fid,J.initial.u,'u');
write_arr(fid,zeros(J.sizes.Nb_in+J.constraints.soft.no,1),'del_u'); %note augment del_u and zeros to avoid separate qp x0
write_arr(fid,NaN(mc,1),'lambda');
write_arr(fid,NaN(mc,1),'tslack');
write_arr(fid,J.initial.xm,'xm');
write_arr(fid,J.initial.del_xm,'del_xm');
%Other
write_var(fid,0,'local_warm','unsigned int');


function writeMPCConstants(fid,J)
%Write global MPC constants to file
global prec addmpc addbytes
addmpc = true; %add mpc bytes
addbytes = true;
statesn = J.sizes.states+J.sizes.n_out;
statesnm = J.sizes.states+J.sizes.nm_out;
statesnq = J.sizes.states+J.sizes.nq_out;
Np_out = J.Np*J.sizes.n_out;
Np_qout = J.Np*J.sizes.nq_out;

fprintf(fid,'//Sizes\n');
write_cvar(fid,J.sizes.n_in,'n_in','int');
write_cvar(fid,J.sizes.Nb_in,'Nb_in','int');
write_cvar(fid,J.sizes.nm_in,'nm_in','int');
write_cvar(fid,J.sizes.n_out,'n_out','int');
write_cvar(fid,J.sizes.nm_out,'nm_out','int');
write_cvar(fid,J.sizes.nm_dist,'nm_dist','int');
write_cvar(fid,J.sizes.nq_out,'nq_out','int');
write_cvar(fid,Np_out,'Np_out','int');
write_cvar(fid,Np_qout,'Np_qout','int');
write_cvar(fid,J.sizes.states,'states','int');
write_cvar(fid,statesn,'statesn','int');
write_cvar(fid,statesnq,'statesnq','int');
write_cvar(fid,statesnm,'statesnm','int');
write_cvar(fid,length(J.index.mex.idelu),'ndelu','int');
write_cvar(fid,length(J.index.mex.iumin),'numin','int');
write_cvar(fid,length(J.index.mex.iumax),'numax','int');
write_cvar(fid,length(J.index.mex.iymin),'nymin','int');
write_cvar(fid,length(J.index.mex.iymax),'nymax','int');

fprintf(fid,'//Indices\n');
write_carr(fid,J.index.mex.iumeasp_out,'iumeasp_out','unsigned int');
write_carr(fid,J.index.mex.ismeasp_out,'ismeasp_out','unsigned int');
write_carr(fid,J.index.mex.isq_out,'isq_out','unsigned int'); 
write_carr(fid,J.index.mex.idelu,'idelu','unsigned int',true); %always write these guys
write_carr(fid,J.index.mex.iumin,'iumin','unsigned int',true);
write_carr(fid,J.index.mex.iumax,'iumax','unsigned int',true);
write_carr(fid,J.index.mex.iymin,'iymin','unsigned int',true);
write_carr(fid,J.index.mex.iymax,'iymax','unsigned int',true);
write_carr(fid,J.index.mex.iman_u,'iman_u','unsigned int');
write_carr(fid,J.index.mex.imeas_dist,'imeas_dist','unsigned int');

fprintf(fid,'//State Estimation\n');
write_carr(fid,J.state_est.Kest','Kest',prec);

fprintf(fid,'//QP & Constraints\n');
write_carr(fid,J.QP.R','R');
write_carr(fid,J.pred.F','F');
if(J.sizes.nq_out ~= J.sizes.n_out)
    write_carr(fid,J.pred.Fq','Fq');
end
write_carr(fid,J.state_est.IKC','IKC');
write_carr(fid,J.QP.S','SE');
write_carr(fid,J.constraints.Tin','Tin');
write_carr(fid,J.QP.PhiTQ','PhiTQ')
if(J.sizes.nm_dist > 0)
    write_carr(fid,J.pred.Phiv','Phiv');
    write_carr(fid,J.pred.Phiqv','Phiqv');
end
write_carr(fid,J.QP.Hscale,'Hscale');
if(~isfield(J.QP,'Ascale') || isempty(J.QP.Ascale))
    write_carr(fid,ones(size(J.constraints.bcon)),'Ascale');
else
    write_carr(fid,J.QP.Ascale,'Ascale');
end
write_carr(fid,J.constraints.bcon,'bcon')
write_cvar(fid,J.constraints.uncon,'uncon','int')
write_cvar(fid,J.constraints.soft.no,'soft','int')
write_cvar(fid,J.mpcopts.QPWarmStart,'warm','int')
if(isempty(J.constraints.u))
    write_carr(fid,-Inf(J.sizes.nm_in,1),'umin');
    write_carr(fid,Inf(J.sizes.nm_in,1),'umax');
    write_carr(fid,Inf(J.sizes.nm_in,1),'delumax')
else
    write_carr(fid,J.constraints.u(:,1),'umin');
    write_carr(fid,J.constraints.u(:,2),'umax');
    write_carr(fid,J.constraints.u(:,3),'delumax')
end


fprintf(fid,'//Other\n');
write_carr(fid,J.lin.u_op,'u_op');
write_carr(fid,J.lin.y_op,'y_op');
write_carr(fid,J.Model.A','modelA');
write_carr(fid,J.Model.B','modelB');


function writeMPCTestGlobals(fid,J,simopts,benchvars)
%Write global MPC Test constants to source file
global addbytes
addbytes = false; %don't count test bytes
fprintf(fid,'// Global Variables //\n');
write_arr(fid,J.initial.up,'upp');
write_arr(fid,simopts.initial.xp,'xp');
write_arr(fid,simopts.initial.yp,'yp');
write_arr(fid,zeros(J.sizes.states+J.sizes.n_out,1),'Kr');

if(benchvars)
    %Intermediate Vars
    write_arr(fid,zeros(max(J.sizes.nm_dist,1),1),'mdistk');   
    write_arr(fid,zeros(J.sizes.nq_out,1),'setpk');    
    write_arr(fid,J.initial.u','mpc_u');
    write_arr(fid,zeros(J.sizes.nm_in,1),'mpc_del_u');    
    write_arr(fid,zeros(J.sizes.states,1),'mpc_xm');    
    write_arr(fid,zeros(J.sizes.n_out,1),'mpc_ym');  
    %Output Saving Vars
    n = simopts.T+1;
    write_arr(fid,zeros(J.sizes.nm_in,n),'pu'); 
    write_arr(fid,zeros(length(simopts.initial.xp),n),'pxp');
    write_arr(fid,zeros(J.sizes.nm_out,n),'pyp');
    write_arr(fid,zeros(1,n),'qpiter','unsigned int');
    write_arr(fid,zeros(1,n),'qpstat','int');
    write_arr(fid,zeros(1,n),'telaps','double');
end

fprintf(fid,'\n// Global Constants //\n');
write_cvar(fid,simopts.T','T','int');
if(~isa(simopts.Plant,'jNL'))
    write_carr(fid,simopts.Plant.A','plantA');
    write_carr(fid,simopts.Plant.B','plantB');
    write_carr(fid,simopts.Plant.C','plantC');
end
write_carr(fid,J.initial.u','init_u');
write_carr(fid,simopts.setp','setp');
write_carr(fid,simopts.udist','umdist');
write_carr(fid,simopts.mdist','mdist');
write_carr(fid,simopts.ydist','ydist');

function writePILGlobals(fid,J,simopts)
%Write global PIL constants to source file
global addbytes
addbytes = false; %don't count pil bytes
fprintf(fid,'// MPC Global Variables //\n');
write_arr(fid,simopts.initial.yp,'yp');
write_arr(fid,NaN(max(J.sizes.nm_dist,1),1),'mdist');   
write_arr(fid,NaN(J.sizes.nq_out,1),'setp');    
write_arr(fid,NaN(J.sizes.nm_in,1),'mpc_u');
write_arr(fid,NaN(J.sizes.nm_in,1),'mpc_del_u');    
write_arr(fid,NaN(J.sizes.states,1),'mpc_xm');    
write_arr(fid,NaN(J.sizes.n_out,1),'mpc_ym');  

fprintf(fid,'// QP Global Variables //\n'); 
write_arr(fid,NaN(size(J.constraints.A,2),1),'QPf');  
write_arr(fid,NaN(size(J.constraints.A,1),1),'QPb'); 
write_arr(fid,NaN(size(J.constraints.A,2),1),'_z');    
write_arr(fid,NaN(size(J.constraints.A,1),1),'_lambda');    
write_arr(fid,NaN(size(J.constraints.A,1),1),'_tslack');  

function write_carr(fid,var,name,dtype,force)
%Write MATLAB variable to C file as a constant array
global prec
if(nargin < 5), force = false; end
if(nargin < 4 || isempty(dtype)), dtype = prec; end
writeVar(fid,var,name,'const',dtype,1,force);

function write_arr(fid,var,name,dtype,force)
%Write MATLAB variable to C file as an array
global prec
if(nargin < 5), force = false; end
if(nargin < 4 || isempty(dtype)), dtype = prec; end
writeVar(fid,var,name,'',dtype,1,force);

function write_cvar(fid,var,name,dtype,force)
%Write MATLAB variable to C file as constant variable
global prec
if(nargin < 5), force = false; end
if(nargin < 4 || isempty(dtype)), dtype = prec; end
writeVar(fid,var,name,'const',dtype,0,force);

function write_var(fid,var,name,dtype,force)
%Write MATLAB variable to C file as normal variable
global prec
if(nargin < 5), force = false; end
if(nargin < 4 || isempty(dtype)), dtype = prec; end
writeVar(fid,var,name,'',dtype,0,force);


function writeVar(fid,var,name,constness,dtype,isptr,force)
%Write MATLAB variable to C file, adding bytes as we go
global strprec constbytes dynbytes mpcbytes qpbytes addbytes addmpc

%Skip if empty and not forcing to write it
if(isempty(var) && ~force), return; end
initglobal = false;

%Determine if writing floating point or integer
switch(dtype)
    case {'long double','double','float'}
        strp = strprec;
    otherwise
        strp = '%d';
end

%Determine size and #bytes of variable
[r,c] = size(var); mul = getsize(dtype);
%Convert To jMPC Typedef
dtype = embedTD(dtype);

%Add constness
if(~isempty(constness)), dtype = sprintf('%s %s',constness,dtype); end
%Update Byte Counter
switch(constness)
    case 'const'
        if(addbytes), constbytes = constbytes + r*c*mul; end
    otherwise
        if(addbytes), dynbytes = dynbytes + r*c*mul; end
        if(~all(isnan(var)))
            initglobal = true;
            if(addbytes), constbytes = constbytes + r*c*mul; end %initialized globals req flash to store initial vals
        end
end
if(addbytes)
    if(addmpc)
        mpcbytes = mpcbytes + r*c*mul;
        if(initglobal), mpcbytes = mpcbytes + r*c*mul; end
    else
        qpbytes = qpbytes + r*c*mul;
        if(initglobal), qpbytes = qpbytes + r*c*mul; end
    end
end

%If all NaNs, uninitialzed global.
if(~isempty(var) && all(all(isnan(var))))
    fprintf(fid,'%s %s[%d];\n',dtype,name,r*c);
    return;
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
%Calculate bytes required to store various data types
global arch
switch(lower(arch))
    case {'generic','arm'}
        switch(type)
            case {'long double','double','long long','unsigned long long'}; mul = 8;
            case {'single','float','long','unsigned long'}; mul = 4;         
            case {'int','unsigned int'}, mul = 2;
            case {'char','unsigned char'}, mul = 1;
            otherwise; mul = 4; %average guess
        end
    case 'c2000'              
        switch(type)
            case {'long double','long long','unsigned long long'}, mul = 8;
            case {'double','single','float','long','unsigned long'}, mul = 4;           
            case {'int','unsigned int','char','unsigned char'}, mul = 2;
            otherwise; mul = 4; %average guess
        end
    otherwise
        error('Unknown architecture - you will need to modify this file manually');
end

function td = embedTD(dtype)
%Get data type type def
switch(dtype)
    case {'long double','double','single','float'}, td = 'realT';        
    case 'int', td = 'intT';
    case 'unsigned int', td = 'uintT';
    case 'long', td = 'longT';
    case 'unsigned long', td = 'ulongT';
    case 'char', td = 'charT';
    case 'unsigned char', td = 'ucharT';
    case 'long long', td = 'llongT';
    case 'unsigned long long', td = 'ullongT';
    otherwise, error('Unknown data type ''%s''',dtype);
end


