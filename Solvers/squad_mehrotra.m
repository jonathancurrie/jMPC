function [z,exitflag,iter,lam,t] = squad_mehrotra(H,f,A,b,maxiter,tol,verbose,z0,lam0,t0)
% Solve quadratic programming problem using Wright's Method & Mehrota's Predictor - Corrector Method [Single Precision]
% Minimise 1/2x'Hx + f'x
% Subject to: Ax <= b 
%
% Reference: Object Orientated Software for Quadratic Programming by E.
% Gertz and S. Wright. University of Wisconsin-Madison.

%   Copyright (C) 2011-2013 Jonathan Currie (www.controlengineering.co.nz)

H = single(H); f = single(f);
A = single(A); b = single(b);

%Length of constraint matrix
mc = single(length(b)); 
%Number of decision variables
ndec = single(length(f));

%Default Args
if(nargin < 10), t = []; else t = single(t0); end
if(nargin < 9), lam = []; else lam = single(lam0); end
if(nargin < 8), z = []; else z = single(z0); end; 
if(nargin < 7 || isempty(verbose)), verbose = 0; end
if(nargin < 6 || isempty(tol)), tol = 1e-4; end
if(nargin < 5 || isempty(maxiter)), maxiter = 200; end
%Test for Warm Start
pmax = max(max(abs([H f; A b])));
if(pmax > 1)
    WARMVAL = single(sqrt(pmax));
else
    WARMVAL = single(0.5);
end
if(isempty(z)) %cold
    z = zeros(ndec,1); 
    lam = WARMVAL*ones(mc,1,'single'); 
    t = WARMVAL*ones(mc,1,'single');   
    wmode = 0;
elseif(isempty(lam)) %just primal
    lam = WARMVAL*ones(mc,1,'single'); 
    t = WARMVAL*ones(mc,1,'single');
    wmode = 1;
elseif(isempty(t)) %primal + dual
    t = WARMVAL*ones(mc,1,'single'); 
    wmode = 2;
else %all
    wmode = 3;
end
tol = single(tol);

%Default Values
ascale = 1; inftol = tol*single(10);
exitflag = 1; cholfail = 0;
mu = t'*lam / mc; At = A';
mr2_1 = single(100); mr2 = single(10); phi1 = single(-Inf); phi = -single(Inf);
%Initial Residuals
r1 = -H*z - At*lam - f;
r2 = -A*z + b;
%Linsolve options
opU.UT = true; opUT.UT = true; opUT.TRANSA = true;

if(verbose)
    fprintf('-----------------------------------------------\n');
    fprintf('QuadMehrotra QP Solver [MATLAB Single Version]\n');
    switch(wmode)
        case 3, fprintf(' Warm Start: Primal + Dual + Slack\n');
        case 2, fprintf(' Warm Start: Primal + Dual\n');
        case 1, fprintf(' Warm Start: Primal\n');
    end
    fprintf(' %4d Decision Variables\n %4d Inequality Constraints\n',ndec,mc);
    fprintf('-----------------------------------------------\n');
    fprintf('iter           phi             mu        sigma        alpha       max(r1)       max(r2)\n');
end 

for iter = 1:maxiter
    %Create common matrices    
    ilam = 1./lam;
    ilamt = ilam.*t;
    lamt = lam./t;   
    IGA = bsxfun(@times,A,lamt); %matrix * diagonal matrix
    igr2 = lamt.*r2;
    RHS = r1+At*igr2;

    %Solve Linear System
    [R,p] = chol(H+At*IGA);
    if(~p)
        del_z = linsolve (R, linsolve (R, RHS, opUT), opU); %exploit matrix properties for solving
    else %Not Positive Definite 
        if(verbose), fprintf(2,'\b (Cholesky Failed)\n'); end
        del_z = (H+At*IGA)\RHS;
        cholfail = cholfail + 1;        
        if(cholfail > 2)
            exitflag = -2;
            break; 
        end
		%Pull back maximum step size
        ascale = ascale - single(0.1);
    end
    del_lam = -igr2+IGA*del_z;
    del_t = -t - ilamt.*del_lam;

    %Decide on suitable affine step-length
    duals = [lam;t];
    delta = [del_lam;del_t];
    index = delta < single(0);
    if(any(index))
        alpha = single(0.9995)*min(-duals(index)./delta(index)); %solves for min ratio (max value of alpha allowed)
    else
        alpha = single(0.999995);
    end
    %Check for numerical problems (alpha will go negative)
    if(alpha < eps(single(1))), exitflag = -3; break; end
    %Local Scaling
    alpha = alpha*ascale;
    %Solve for Centering Variable
    mu1 = dot(lam+alpha*del_lam,t+alpha*del_t)/mc;
    sigma = min((mu1/mu)^3,single(0.99999));    

    %Solve for Correction (2nd Derivative)
    term = (sigma*mu - del_lam.*del_t)./t;
    RHS  = RHS - A'*term;
    if(~p)
        del_z = linsolve (R, linsolve (R, RHS, opUT), opU);
    else
        del_z = (H+At*IGA)\RHS;
    end
    del_lam = -igr2+IGA*del_z + term;
    del_t = -A*del_z + r2 - t;

    %Decide on suitable corrector step-length
    duals = [lam;t];
    delta = [del_lam;del_t];
    index = delta < single(0);
    if(any(index))
        alpha = single(0.9995)*min(-duals(index)./delta(index)); %solves for min ratio (max value of alpha allowed)
    else
        alpha = single(0.999995);
    end
    %Check for numerical problems (alpha will go negative)
    if(alpha < eps(single(1))), exitflag = -3; break; end
    %Local Scaling
    alpha = alpha*ascale;
    %Sum Increments
    z   = z + alpha*del_z;
    lam = lam + alpha*del_lam;
    t   = t + alpha*del_t;
    
    %Update residuals
    r1 = (1-alpha)*r1; %equiv to r1 = -H*z - At*lam - f
    r2 = -A*z + b;
    %Complementarity Gap
    mu = t'*lam / mc;
    %Infeasibility Phi
    mr2_2 = mr2_1; mr2_1 = mr2;
    mr1 = max(abs(r1)); mr2 = max(abs(r2-t));
    phi2 = phi1; phi1 = phi;
    phi = (max([mr1,mr2]) + t'*lam)/pmax;
    if(verbose)
        fprintf('%3d  %13.5g  %13.5g  %11.5g  %11.5g   %11.5g   %11.5g\n',iter,phi,mu,sigma,alpha,mr1,mr2);
    end
    %Check for NaNs
    if(isnan(mu) || isnan(phi)), exitflag = -3; break; end
    %Check Convergence
    if(mu <= tol && phi<=tol)
        exitflag = 1;
        if(verbose)
            fprintf('-----------------------------------------------\n');
            fprintf(' Successfully solved in %d Iterations\n Final phi: %d, mu %g [tol %g]\n',iter,phi,mu,tol);
            fprintf('-----------------------------------------------\n');
        end
        return
    end
    %Check For Primal Infeasible
    if(iter > 4 && mr2/pmax > tol/single(10))
        if((phi > phi1 && phi1 > phi2) || (abs(mr2-mr2_1)/mr2 < inftol && abs(mr2_1-mr2_2)/mr2_1 < inftol))
            if(verbose), fprintf(2,'\b (Primal Infeasibility Detected)\n'); end  
            exitflag = -4;
            break;
        end  
    end
end

%If here, either bailed on Cholesky or Iterations Expired
if(exitflag==1), exitflag = -1; end %expired iters    
%Optional Output
if(verbose)
    fprintf('-----------------------------------------------\n');
    switch(exitflag) 
        case -1, fprintf(' Maximum Iterations Exceeded\n');
        case -2, fprintf(2,' Failed - Cholesky Factorization Reported Errors\n');
        case -3, fprintf(2,' Failed - Numerical Errors Detected\n'); 
        case -4, fprintf(2,' Failed - Problem Looks Infeasible\n');
    end
    fprintf(' Final phi: %g, mu %g [tol %g]\n',phi,mu,tol);
    fprintf('-----------------------------------------------\n');
end
