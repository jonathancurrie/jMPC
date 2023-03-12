%% Initial Test
clc
% Run
H = eye(3);
f = [-2;-8;-1];
A = [1 1 1 ; 3 -2 -3 ; 1 -3 2 ; 2 -3 4];   %Aeq
b = [1;1;1;1]; 

[xmfile,exitflag,iter] = quad_wright(H,f,A,b,50,1e-6,[])
[xmex,exitflag,iter] = mquad_wright(H,f,A,b,50,1e-6)
acc = log10(norm(xmfile-xmex))

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
        xqp = quad_wright(H,f,A,b,200,1e-6,[]);
        Tqp = toc(tqps);

        %Solve Using Mex Wright
        tws = tic; 
        xw = mquad_wright(H,f,A,b,200,1e-6);
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
mc = 100;
ndec = 100;

%Random problem
Tr = randn(ndec); 
H = Tr\diag(1:ndec)*Tr; 
H = H'*H; f = 3+(1:ndec)';        
%Random constraints
A = randn(mc,ndec); b = randn(mc,1); 

[xmfile,exitflag,iter] = quad_wright(H,f,A,b,50,1e-6,[])
[xmfile2,exitflag,iter1] = quad_mehrotra(H,f,A,b,50,1e-6,[])
[xmex,exitflag,iter2] = mquad_wright(H,f,A,b,50,1e-6)
iter
iter1
acc = log10(norm(xmfile-xmex))