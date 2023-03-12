function [del_u,stats,warm] = solve_qp(J,b,f,k,tstart,forceQP)
%Solve Quadratic Programming Problem for MPC Control Action
%
%   Called By jMPC\solve_input

%   Jonathan Currie (C)
%   Control Engineering 2013

%Exitflags
% 1 - QP solved successfully
% 0 - Global solution is valid
% -1 - Maximum iterations reached
% -2 - Cholesky Failed
% -3 - Numerical Errors
% -4 - Infeasible
% -5 - Slow Progress
% -6 - Unknown exit

%Warm Starting Variables
persistent z lam t oasesQP
if(k<=1 || ~J.mpcopts.QPWarmStart) %first sample, re-initialize
    z = [];
    lam = [];
    t = [];    
    oasesQP = [];
end
if(nargin < 6), forceQP = false; end
if(nargin < 5), tstart = tic; end

%Get Warning Level
if(strcmpi(J.mpcopts.Warnings,'all'))
    warn = 2;
elseif(strcmpi(J.mpcopts.Warnings,'critical'))
    warn = 1;
else
    warn = 0;
end

%Default Stats Structure
stats = struct('qpiter',0,'status',0,'tglobal',0,'tqp',0);
warm = struct('z',[],'lam',[],'t',[]);
%Get Serial Device
if(isfield(J.mpcopts,'SerialDevice'))
    sd = J.mpcopts.SerialDevice;
else
    sd = [];
end

%Disable Singular Matrix Warnings
warningstate1 = warning('off', 'MATLAB:nearlySingularMatrix');
warningstate2 = warning('off', 'MATLAB:singularMatrix');

%Calculate Global Minimum
x = -J.QP.R\(J.QP.R'\f); % -H\f;

%Check Problem is Constrained
if(J.constraints.uncon)
    stats.tgobal = toc(tstart);
    stats.tqp = toc(tstart);
    del_u = x;         
%Constrained Problem
else
    if(~forceQP)
        %Check Global Minimum isn't also Constrained Minimum
        gl = any(J.constraints.A*x > b); stats.tglobal = toc(tstart);
    else
        gl = true;
    end
    %Global minimum satisfies constraints
    if(~gl) 
        del_u = x; 
        stats.tqp = toc(tstart);
        z = x; lam = []; t = []; %copy global primal solution for qp solver, remove dual
    %Use Constrained Solver
    else 
        %Use user selected iter and tol
        maxiter = J.mpcopts.QPMaxIter;
        tol = J.mpcopts.QPTol;
        qpverb = J.mpcopts.QPVerbose;
        %Save warm start vars
        warm.z = z; warm.lam = lam; warm.t = t;
        if(qpverb), fprintf('Sample %d:\n',k); end

        %Switch depending on selected QP solver
        switch(lower(J.mpcopts.QPSolver))
            case 'quad_mehrotra', [del_u,exitflag,iter,lam,t] = quad_mehrotra(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;                                 
            case 'mquad_mehrotra', [del_u,exitflag,iter,lam,t] = mquad_mehrotra(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;            
            case 'squad_mehrotra', [del_u,exitflag,iter,lam,t] = squad_mehrotra(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;
            case 'msquad_mehrotra', [del_u,exitflag,iter,lam,t] = msquad_mehrotra(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;

            case 'quad_wright', [del_u,exitflag,iter,lam,t] = quad_wright(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u; 
            case 'mquad_wright', [del_u,exitflag,iter,lam,t] = mquad_wright(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;            
            case 'squad_wright', [del_u,exitflag,iter,lam,t] = squad_wright(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;            
            case 'msquad_wright', [del_u,exitflag,iter,lam,t] = msquad_wright(J.QP.H,f,J.constraints.A,b,maxiter,tol,qpverb,z,lam,t); z = del_u;
                                
            case 'qpoases'
                if(qpverb > 0), qpverb = 2; end
                opts = qpOASES_options('MPC','printLevel',qpverb,'maxIter',maxiter,'terminationTolerance',tol);
                if(J.mpcopts.QPWarmStart)
                    if(isempty(oasesQP)) %first solve initialize too
                        [oasesQP,del_u,~,ef,iter] = qpOASES_sequence('i',J.QP.H,f,J.constraints.A,[],[],[],b,opts);
                    else %update and solve
                        [del_u,~,ef,iter] = qpOASES_sequence( 'h',oasesQP,f,[],[],[],b,opts);
                    end
                %NON-sequential version
                else
                    [del_u,~,ef,iter] = qpOASES(J.QP.H,f,J.constraints.A,[],[],[],b,opts);                
                end
                switch(ef)
                    case 0, exitflag = 1;
                    case 1, exitflag = -1;
                    case -2, exitflag = -4;
                    otherwise, exitflag = -6;
                end
                
            case 'quad_hildreth', [del_u,exitflag,iter] = quad_hildreth(J.QP.H,f,J.constraints.A,b,maxiter,tol);

            case 'quadprog'
                [del_u,fval,exitflag,output] = quadprog(J.QP.H,f,J.constraints.A,b,[],[],[],[],z,J.mpcopts.QuadProgOpts); z = del_u; %#ok<ASGLU>
                iter = output.iterations;                
                switch(exitflag)
                    case 0, exitflag = -1;
                    case 4, exitflag = 1;
                    case {-2,-3}, exitflag = -4;                    
                    case {-6,3,-4,-7}, exitflag = -3;
                end
                
            case 'qpip'
                [del_u,ef] = qpip(J.QP.H,f,J.constraints.A,b,[],[],[],[],qpverb); iter = NaN;
                switch(ef)
                    case 0, exitflag = 1;
                    otherwise, exitflag = -5;
                end
            case 'qpas'
                [del_u,ef] = qpas(J.QP.H,f,J.constraints.A,b,[],[],[],[],qpverb); iter = NaN;
                switch(ef)
                    case 0, exitflag = 1;
                    otherwise, exitflag = -5;
                end
                
            case 'clp'
                opts = struct('primalTol',tol,'dualTol',tol,'maxiter',maxiter,'display',qpverb);
                [del_u,~,ef,iter] = clp(J.QP.spH,f,J.constraints.spA,-Inf(size(b)),b,[],[],opts);
                switch(ef)
                    case 0, exitflag = 1;
                    case {1,2}, exitflag = -4;
                    case 3, exitflag = -1;
                    otherwise, exitflag = -5;
                end
                
            case 'ooqp'
                opts = struct('maxiter',maxiter,'display',qpverb);
                [del_u,~,ef,iter] = ooqp(J.QP.spH,f,J.constraints.spA,-Inf(size(b)),b,[],[],[],[],opts);
                switch(ef)
                    case 0, exitflag = 1;
                    case 2, exitflag = -1;
                    case 3, exitflag = -4;
                    otherwise, exitflag = -5;
                end
                
            case 'pil'
                if(isempty(sd))
                    error('You must supply a serial port object via ''serialdevice'' to jSIM');
                end
                if(J.mpcopts.Single)
                    dtype = 'single';
                else
                    dtype = 'double';
                end
                %See if we have z, lam, t
                if(~isempty(z))
                    if(~isempty(lam))
                        if(~isempty(t))
                            warm = 3;
                        else
                            t = zeros(size(J.QP.Ascale),dtype);
                            warm = 2;
                        end
                    else
                        lam = zeros(size(J.QP.Ascale),dtype);
                        t = zeros(size(J.QP.Ascale),dtype);
                        warm = 1;
                    end
                else
                    z = zeros(size(J.QP.Hscale),dtype);
                    lam = zeros(size(J.QP.Ascale),dtype);
                    t = zeros(size(J.QP.Ascale),dtype);
                    warm = 0;
                end
                if(~J.mpcopts.QPWarmStart), warm = 0; end
                if(J.mpcopts.Single)
                    z = single(z); lam = single(lam); t = single(t);
                end
                %Transmit Start Info
                fwrite(sd,uint8(warm));
                jMPC_SerialTx(sd,f); jMPC_SerialTx(sd,b);
                jMPC_SerialTx(sd,z); jMPC_SerialTx(sd,lam); jMPC_SerialTx(sd,t);
                %Receive Solution
                del_u = jMPC_SerialRx(sd,length(f),class(f));
                lam = jMPC_SerialRx(sd,length(b),class(b));
                t = jMPC_SerialRx(sd,length(b),class(b));
                Te = jMPC_SerialRx(sd,1,'uint32');
                iter = jMPC_SerialRx(sd,1,'uint16');
                exitflag = jMPC_SerialRx(sd,1,'int16');
                z = del_u;
            otherwise
                error('Unknown QP Solver %s\n',J.mpcopts.QPSolver);
        end
        
        stats.tqp = toc(tstart);
        stats.qpiter = iter;
        stats.status = exitflag;
        if(J.mpcopts.QPWarmStart)
            if(~isempty(lam))
                ind = lam < 0.1;
                lam(ind) = lam(ind) + 0.15; %add a little so solvers can find new active constraints
            end
            if(~isempty(t))
                ind = t < 0.1;
                t(ind) = t(ind) + 0.15;
            end
        end
        %Check for single solver with double sim
        if(isa(del_u,'single') && ~J.mpcopts.Single)
            del_u = double(del_u);
        end

        %Display Status If Required
        if(warn && qpverb==0)
            switch(exitflag)
                case -1                            
                    fprintf('Sample %d: Maximum Iterations Exceeded\n',k);
                case -2
                    fprintf('Sample %d: Cholesky Failure\n',k);                            
                case -3
                    fprintf('Sample %d: Numerical Errors Encountered\n',k);
                case -4
                    fprintf('Sample %d: No Feasible Solution Found\n',k);
                case -5
                    fprintf('Sample %d: Slow Progress - Early Exit\n',k);
                case -6
                    fprintf('Sample %d: Unknown QP Exit\n',k);
            end
        end
    end            
end
%Catch Numerical Errors
ind = isnan(del_u);
if(any(ind))
    del_u(isnan(del_u)) = 0; %return to 'safe' value
    stats.status = -3;
end
% Restore the warning states to their original settings
warning(warningstate1)
warning(warningstate2)
%If soft constrained, remove the final slack variable(s)
if(J.constraints.soft.no)
    del_u = del_u(1:end-J.constraints.soft.no);
end
