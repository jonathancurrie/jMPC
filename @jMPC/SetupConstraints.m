function con = SetupConstraints(con,Phi,Np,Nb,nm_in,n_out,opts)
%Collect MPC Constraints and form Inequality Constraints
%
%   Copyright (C) 2011-2013 Jonathan Currie (www.i2c2.aut.ac.nz)

Nbin = Nb*nm_in;

%Get Warning Level
if(strcmpi(opts.Warnings,'all'))
    warn = 2;
elseif(strcmpi(opts.Warnings,'critical'))
    warn = 1;
else
    warn = 0;
end
if(~isfield(con,'u')), con.u = []; end
if(~isfield(con,'y')), con.y = []; end
if(~isfield(con,'slack')), con.slack = []; end
if(isfield(con,'soft')), con.slack = con.soft; end

%Build Default Return Structure
con = struct('u',con.u,'y',con.y,'uncon',true,'delucon',false,'ucon',false,'ycon',false,...
             'A',[],'bcon',[],'Tin',[],'indices',struct('idelu',[],'iumin',[],'iumax',[],'iymin',[],'iymax',[]),...
             'slack',con.slack,'soft',struct('no',0,'ind',[]));

%Determine if Constrained
if(~isempty(con.u))
    if(~all(all(isinf(con.u(:,end)))))
        con.delucon = true;
    else
        con.delucon = false;
    end
    if(~all(all(isinf(con.u(:,1:2)))))
        con.ucon = true;
    else
        con.ucon = false;
    end
end
if(~isempty(con.y) && ~all(all(isinf(con.y))))
    con.ycon = true;
else
    con.ycon = false;
end
if(con.delucon || con.ucon || con.ycon)
    con.uncon = false;
else
    return; %nothing else to do here
end

%Determine if Soft Constraints Enabled
if(isfield(con,'slack'))    
    con.soft.ind = ~isinf(con.slack);
    con.soft.no = sum(con.soft.ind); %record only finite slack weights
end
% Check for large constraint values (causes numerical problems in qp)
if(con.delucon && any(con.u(~isinf(con.u(:,end)),end) > 1e3) && warn)
    jmpcwarn('jMPC:NumericalIssue','One or more input rate constraints are > 1e3. This can cause numerical problems in the solver!');
end
if(con.ucon && any(any(con.u(~isinf(con.u(:,2)),2) > 1e3)) && warn)
    jmpcwarn('jMPC:NumericalIssue','One or more upper input constraints are > 1e3. This can cause numerical problems in the solver!');
end
if(con.ucon && any(any(con.u(~isinf(con.u(:,1)),1) < -1e3)) && warn)
    jmpcwarn('jMPC:NumericalIssue','One or more lower input constraints are < -1e3. This can cause numerical problems in the solver!');
end
if(con.ycon && any(any(con.y(~isinf(con.y(:,2)),2) > 1e3)) && warn)
    jmpcwarn('jMPC:NumericalIssue','One or more upper output constraints are > 1e3. This can cause numerical problems in the solver!');
end
if(con.ycon && any(any(con.y(~isinf(con.y(:,1)),1) < -1e3)) && warn)
    jmpcwarn('jMPC:NumericalIssue','One or more lower output constraints are < -1e3. This can cause numerical problems in the solver!');
end

%Build up Constraints
A = []; b = [];
if(con.delucon)
    delumax = con.u(:,3); ildelu = ~isinf(delumax); idelu = repmat(ildelu,Nb,1);
    %Expand to the length of Nb
    delumax = repmat(delumax(ildelu),Nb,1);
    I = eye(Nbin); I = I(idelu,:);
    if(con.soft.no)
        A = [[I;-I] zeros(size(I,1)*2,con.soft.no)]; 
    else
        A = [I;-I];
    end
    b = [delumax;delumax];
    %Save indices
    con.indices.idelu = idelu;
else
    con.indices.idelu = false(Nbin,1);
end

if(con.ucon)
    umin = con.u(:,1); ilumin = ~isinf(umin); iumin = repmat(ilumin,Nb,1);
    umax = con.u(:,2); ilumax = ~isinf(umax); iumax = repmat(ilumax,Nb,1);
    %Expand to the length of Nb
    umin = repmat(umin(ilumin),Nb,1);
    umax = repmat(umax(ilumax),Nb,1);
    %Form lower triangular matrix for use in updateRHS and below
    T = tril(repmat(eye(nm_in),Nb,Nb)); Tl = T(iumin,:); Tu = T(iumax,:);
    con.Tin = T(:,1:nm_in);
    %Only save the finite u constraints
    if(any(ilumin))    
        if(con.soft.no)
            A=[A;-Tl zeros(length(umin),con.soft.no)]; %never soft cons on u
        else
            A=[A;-Tl];
        end
        b=[b;-umin];
    end
    if(any(ilumax))   
        if(con.soft.no)
            A=[A;Tu zeros(length(umax),con.soft.no)];
        else
            A=[A;Tu];
        end
        b=[b;umax];
    end
    %Save indices
    con.indices.iumin = iumin;
    con.indices.iumax = iumax;
else
    con.indices.iumin = false(Nbin,1);
    con.indices.iumax = false(Nbin,1);
end

if(con.ycon)
    ymin = con.y(:,1); ilymin = ~isinf(ymin); iymin = repmat(ilymin,Np,1);
    ymax = con.y(:,2); ilymax = ~isinf(ymax); iymax = repmat(ilymax,Np,1);
    %Expand to the length of Np
    ymin = repmat(ymin(ilymin),Np,1);
    ymax = repmat(ymax(ilymax),Np,1);    
    %Only save the finite y constraints
    if(any(ilymin))   
        if(con.soft.no)
            A=[A;-Phi(iymin,:) genSoftDiag(ilymin,con.soft,Np)]; 
        else
            A=[A;-Phi(iymin,:)];
        end
        b=[b;-ymin];
    end
    if(any(ilymax)) 
        if(con.soft.no)
            A=[A;Phi(iymax,:) genSoftDiag(ilymax,con.soft,Np)]; 
        else
            A=[A;Phi(iymax,:)];
        end
        b=[b;ymax];
    end  
    %Save indices
    con.indices.iymin = iymin;
    con.indices.iymax = iymax;
else
    con.indices.iymin = false(Np*n_out,1);
    con.indices.iymax = false(Np*n_out,1);
end

% %Find and remove empty row constraints
% fullRow = true(size(A,1),1);
% for i = 1:size(A,1)
%     if(all(A(i,1:Nbin) == 0))
%         fullRow(i) = false;
%     end
% end
% A = A(fullRow,:); b = b(fullRow);

%Save constraints
con.A = A;      %LHS Inequalities
con.bcon = b;   %RHS Inequalities (constant part)
end


function sd = genSoftDiag(index,soft,Np)
%Generate the slack variable constraint LHS
sd = -1*eye(length(index));
sd = repmat(sd(index,soft.ind),Np,1);
end