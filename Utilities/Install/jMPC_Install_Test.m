function ok = jMPC_Install_Test(varargin)
% Installation Test File
%
% A collection of tests to verify the installation of the toolbox has been
% successful. 

%   Jonathan Currie (C)
%   AUT University 2009

%Set Verbosity
if(nargin < 1)
    verb = 1; %default show all messages
else
    verb = varargin{1};
end

%Set default ok
ok = 1;

fprintf('\nChecking jMPC Toolbox Installation:\n');
ext = ['.' mexext];

%% TEST 1 - Check Main Paths
if(verb); fprintf('Checking Paths... '); end
paths = {'@jMPC','@jSIM','@jNL','@jGUI','@ScrollPlot','GUI','QP Solvers','Simulink','Help','jMPC_Tests'};
len = length(paths);
for i = 1:len
    pass = exist(paths{i},'dir');
    if(~pass)
        fprintf('\nFailed Path Check on %s\n',paths{i});
        ok = 0;
        return
    end
end
if(verb); fprintf('Ok\n'); end

%% TEST 2 - Check QP Solvers Exist
if(verb); fprintf('Checking QP Solvers Exist... '); end
files = {'quad_wright.m','quad_mehrotra.m',['mquad_wright',ext],...
         ['mquad_wright',ext],['mquad_mehrotra',ext],['mquad_mehrotra',ext]};
len = length(files);
for i = 1:len
    pass = exist(files{i},'file');
    if(~pass)
        fprintf('\nFailed QP Check on %s\n',files{i});
        if(files{i}(end) ~= 'm')
            fprintf('Please Run jMPC_Mex_Install.m to Compile MEX solvers\n');
        end
        ok = 0;
        return
    end
end
if(verb); fprintf('Ok\n'); end


%% TEST 3 - Check QP Solver Results
if(verb); fprintf('Checking QP Solver Results... '); end
load qp_test %load pre solved MPC QP (solved using quadprog)

solvers = {'quad_wright','quad_mehrotra'};
if(ispc), solvers = [solvers,'mquad_wright','mquad_mehrotra']; end

%Cold start test
len = length(solvers);
for i = 1:len
    xnew = eval([solvers{i},'(H,f,A,b,200,1e-9)']);
    if(norm(x-xnew) > 1e-5) 
        fprintf('\nFailed QP Cold Start Result Check on %s\n',solvers{i});
        ok = 0;
        return
    end
end

%Warm start test
len = length(solvers);
for i = 1:len
    xnew = eval([solvers{i},'(H,f,A,b,200,1e-9,0,z,lam,t)']);
    if(norm(xw-xnew) > 1e-5) 
        fprintf('\nFailed QP Warm Start Result Check on %s\n',solvers{i});
        ok = 0;
        return
    end
end
if(verb); fprintf('Ok\n'); end   

%% TEST 4 - Check Matlab jMPC Results
if(verb); fprintf('Checking Matlab jMPC Results... '); end
no_test = 7;
for i = 1:no_test
    eval(['load mpc_test',num2str(i)]); opts.Warnings = 'none'; opts.WaitBar = 0; opts.QPSolver = 'quad_mehrotra';
    MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
    simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);
    simresult_new = sim(MPC1,simopts,'Matlab');
    if(norm(simresult.plotvec.yp - simresult_new.plotvec.yp) > 0.5)
        fprintf('\nFailed Matlab MPC Result Check on Test %d\n',i);
        ok = 0;
        return    
    end
end
if(verb); fprintf('Ok\n'); end  

% %% TEST 5 - Check Simulink jMPC Results
% mver = ver('matlab'); doTest = true;
% vv = regexp(mver.Version,'\.','split');
% if(str2double(vv{1}) < 8)
%     if(str2double(vv{2}) <= 14) %2012b
%         doTest = false;
%     end
% end
% if(exist('jMPC_sfcn','file')==3 && doTest)
%     sk = which('simset.m');
%     if(~isempty(sk))
%         ws = warning('query','Simulink:Commands:LoadingNewerModel');
%         warning('off','Simulink:Commands:LoadingNewerModel');
%         if(verb); fprintf('Checking Simulink jMPC Results [Starting Simulink, Please Wait]... '); end
%         no_test = 9; %final test differences too large
%         for i = 1:no_test
%             eval(['load mpc_test',num2str(i)]); opts.Warnings = 'none'; opts.WaitBar = 0;
%             MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
%             simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);
%             simresult_new = sim(MPC1,simopts,'Simulink');
%             if(norm(simresult.plotvec.yp - simresult_new.plotvec.yp) > 1e-3)
%                 fprintf('\nFailed Simulink MPC Result Check on Test %d\n',i);
%                 ok = 0;
%                 warning(ws);
%                 return    
%             end
%         end
%         if(verb); fprintf('Ok\n'); end 
%         warning(ws);
%     end
% elseif(doTest)
%     if(verb); fprintf('Checking Simulink jMPC Results... Not Available On This Operating System\n'); end    
% end

%% TEST 6 - Check MEX jMPC Results
if(exist('jMPCEngine','file')==3)
    if(verb); fprintf('Checking MEX jMPC Results... '); end
    no_test = 6; %linear only
    for i = 1:no_test
        eval(['load mpc_test',num2str(i)]); opts.Warnings = 'none'; opts.WaitBar = 0;
        MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
        simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);
        simresult_new = sim(MPC1,simopts,'Mex');
        if(norm(simresult.plotvec.yp - simresult_new.plotvec.yp) > 1e-3)
            fprintf('\nFailed MEX MPC Result Check on Test %d\n',i);
            ok = 0;
            return    
        end
    end
    if(verb); fprintf('Ok\n'); end 
else
    if(verb); fprintf('Checking MEX jMPC Results... Not Available On This Operating System\n'); end
end

%% Final
fprintf('Toolbox Checked Out Ok! - Enjoy\n');