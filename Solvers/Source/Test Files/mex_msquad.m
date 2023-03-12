%% Initial Test
clc
% Run
H = eye(3);
f = [-2;-8;-1];
A = [1 1 1 ; 3 -2 -3 ; 1 -3 2 ; 2 -3 4];   %Aeq
b = [1;1;1;1]; 

[xmex,exitflag,iter] = squad_wright(H,f,A,b,50,1e-4)
[xsmex,exitflag,iter] = msquad_wright(H,f,A,b,50,1e-4)
acc = log10(norm(xmex-xsmex))

%% Full Comparison

% if(matlabpool('size') < 1)
%     matlabpool %start parallel toolbox
% end
% while(matlabpool('size') < 1); end

clear

%Testing Setup
no_dec = 10:10:100;
%no_con = 10;
no_montec = 30;
%reset(RandStream.getDefaultStream); %Use for comparison

%Find length
no_sim = length(no_dec);
%Preallocate
D = zeros(no_montec,1);
T = zeros(no_montec,1);
R = zeros(no_sim,2);
Tav = zeros(no_sim,1);
h = waitbar(0,'Running...');

tsolve = tic;
for m = 1:no_sim
    parno_dec = no_dec(m);
    no_con = parno_dec+10;
    parfor n = 1:no_montec %use parallel labs for monte carlo
        %Random problem
        Tr = randn(parno_dec); 
        H = Tr\diag(1:parno_dec)*Tr; 
        H = H'*H; f = 3+(1:parno_dec)';        
        %Random constraints
        A = randn(no_con,parno_dec); b = randn(no_con,1); 

        %Solve Using Original Wright
        tqps = tic;
        xqp = squad_wright(H,f,A,b,200,1e-4,[]);
        Tqp = toc(tqps);

        %Solve Using Mex Wright
        tws = tic; 
        xw = msquad_wright(H,f,A,b,200,1e-4);
        Tw = toc(tws);   
        
        %Find norm of differences
        D(n,:) = log10(norm(xqp-xw));
        %Collect Times
        T(n) = Tqp/Tw; 
    end
    waitbar(m/no_sim,h);
    %Determine std & mean of error
    R(m,:) = [nanmean(D), std(D)]; 
    %Determine average of time
    Tav(m) = nanmean(T);
end
clc
close(h);
toc(tsolve)

subplot(211)
errorbar(no_dec,R(:,1),R(:,2),'r');
ylabel('Log10 Accuracy'); title('M Wright vs C Wright Comparison');
axis([min(no_dec) max(no_dec) ylim]);

subplot(212)
plot(no_dec,Tav,'*-'); hold on; plot(xlim,[1 1],'k:'); hold off;
ylabel('Time M / Time C'); title('Solve Time'); xlabel('No Dec Variables');
axis([min(no_dec) max(no_dec) ylim]);

fprintf('\nAverage Speed Increase: %3.3g\n',mean(Tav));
fprintf('\nAverage Accuracy: %3.3g\n',mean(R(:,1)));

%%
clc
mc = 100;
ndec = 100;

%Random problem
Tr = randn(ndec); 
H = Tr\diag(1:ndec)*Tr; 
H = H'*H; f = 3+(1:ndec)';        
%Random constraints
A = randn(mc,ndec); b = randn(mc,1); 
%%
clc
niter = 30; tol = 1e-4;
% [xmfile,exitflag,iter] = quad_wright(H,f,A,b,niter,tol,1);
[xmfile,exitflag,iter2] = mquad_wright(H,f,A,b,niter,tol,1);
% [xmfile,exitflag,iter1] = squad_wright(H,f,A,b,niter,tol,1,1);
[xmex,exitflag2,iter3] = msquad_wright(H,f,A,-b,niter,tol,1);
quadprog(H,f,A,-b,[],[],[],[],[],optimset('algorithm','interior-point-convex'))
% iter
% iter1
% iter2
% % exitflag
% iter2
% exitflag2
acc = log10(norm(xmfile-xmex))


%%
clc
clear
QPs = mpc_qps(5,2,3,3,5,1,0);
H = QPs.H(:,:,1); f = QPs.f(:,1);
A = QPs.A(:,:,1); b = QPs.b(:,1);
niter = 30; tol = 1e-7;

%%
clc
Hfac = 1./max((H))*1;
Afac = 1/max(max(A))*1;

R = diag(Afac*ones(size(b)));
AR = R*A; Rb = R*b;
R2 = diag(Hfac*ones(size(f)));
HR = R2*H; Rf = R2*f;

% [z,~,~,lam] = mquad_wright(H,f,A,b,niter,tol,1);
[xmfile1,exitflag,iter1] = squad_wright(H,f,A,b,niter,tol,1); 
[xmfile2,exitflag,iter2] = squad_wright(HR,Rf,AR,Rb,niter,tol,1); 
[xmfile3,exitflag,iter3] = quad_wright(H,f,A,b,niter,tol,0); 
[xmfile4,exitflag,iter4] = quad_wright(HR,Rf,AR,Rb,niter,tol,0); 
% %%
% sng_noscal_iter = iter1
% sng_scal_iter = iter2
% dbl_noscal_iter = iter3
% dbl_scal_iter = iter4

sng_acc = log10(norm(xmfile1-xmfile2))
dbl_acc = log10(norm(xmfile3-xmfile4))

%%
clc
[xmfile,exitflag,iter] = squad_wright(H,f,A,b,niter,tol,1);

% [z,~,~,lam] = mquad_wright(H,f,A,b,niter,tol,1);
[xmfile2,exitflag2,iter1] = mquad_wright(H,f,A,b,niter,tol,1);

acc = log10(norm(xmfile-xmfile2))
