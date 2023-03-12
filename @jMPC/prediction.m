function pred = prediction(Model,QPModel,Np,Nc,Nb,index)
%Create MPC Prediction Matrices
%
%   Called By buildMPC

%   Jonathan Currie (C)
%   Control Engineering 2011

%Extract ss Data
[A,B,C] = ssdata(Model);
[Aq,Bq,Cq] = ssdata(QPModel);

%Collect index vectors
imeas_dist = index.imeas_dist;
iman_u = index.iman_u;

%Collect sizes
n_out = size(C,1);
[statesn,n_in] = size(B);
nq_out = size(Cq,1);
statesnq = size(Bq,1);
nm_dist = length(imeas_dist);

%Preallocate
F = zeros(Np*n_out,statesn);
Fq = zeros(Np*nq_out,statesnq);
h = zeros(Np*n_out,statesn);
hq = zeros(Np*nq_out,statesnq);
if(nm_dist > 0)
    Nbnin = Nb*(n_in-nm_dist);   
else
    Nbnin = Nb*n_in;
end
Phi = zeros(Np*n_out,Nbnin);
Phiq = zeros(Np*nq_out,Nbnin);
Phiv = zeros(Np*n_out,max(nm_dist,1));
Phiqv = zeros(Np*nq_out,max(nm_dist,1));
n_in = length(iman_u);

%Initialise
F(1:n_out,:) = C*A;
Fq(1:nq_out,:) = Cq*Aq;
h(1:n_out,:) = C;
hq(1:nq_out,:) = Cq;

%Create F
for i = n_out+1:n_out:Np*n_out
    F(i:i+n_out-1,:) = F(i-n_out:i-1,:)*A;
    h(i:i+n_out-1,:) = h(i-n_out:i-1,:)*A;
end
for i = nq_out+1:nq_out:Np*nq_out
    Fq(i:i+nq_out-1,:) = Fq(i-nq_out:i-1,:)*Aq;
    hq(i:i+nq_out-1,:) = hq(i-nq_out:i-1,:)*Aq;
end

%Create Measured Disturbance Prediction
if(nm_dist > 0)    
    Phiv(:,1:nm_dist) = h*B(:,imeas_dist);
    Phiqv(:,1:nm_dist) = hq*Bq(:,imeas_dist);
end

%Shift & Insert Phi, blocking as we go
h = h*B(:,iman_u);
Phi(:,1:n_in) = h;
col = n_in+1;
Npout = Np*n_out;

hq = hq*Bq(:,iman_u);
Phiq(:,1:n_in) = hq;
colq = n_in+1;
Npqout = Np*nq_out;

if(length(Nc) > 1) %Blocking
    row = n_out*Nc(1);
    rowq = nq_out*Nc(1);
    for i = 2:Nb   
        %Full Prediction
        Phi(:,col:col+n_in-1) = [zeros(row,n_in); h(1:Npout-row,1:n_in)];
        col = col+n_in;
        row = row+n_out*Nc(i);
        %Controlled Prediction
        Phiq(:,colq:colq+n_in-1) = [zeros(rowq,n_in); hq(1:Npqout-rowq,1:n_in)];
        colq = colq+n_in;
        rowq = rowq+nq_out*Nc(i);
    end   
else
    row = n_out;
    rowq = nq_out;
    for i = 2:Nb 
        %Full Prediction
        Phi(:,col:col+n_in-1) = [zeros(row,n_in); h(1:Npout-row,1:n_in)];
        col = col+n_in;
        row = row+n_out;
        %Controlled Prediction
        Phiq(:,colq:colq+n_in-1) = [zeros(rowq,n_in); hq(1:Npqout-rowq,1:n_in)];
        colq = colq+n_in;
        rowq = rowq+nq_out;
    end
end

%Creating prediction structure
pred.F = F;
pred.Fq = Fq;
pred.Phi = Phi;
pred.Phiq = Phiq;
pred.Phiv = Phiv;
pred.Phiqv = Phiqv;

end