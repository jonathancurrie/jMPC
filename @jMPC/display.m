function display(J)
%Display jMPC Setup Information
%
%   Called By jMPC Class

%   Jonathan Currie (C)
%   AUT University 2009

%Distribute Structure Variables;
Model = J.StdModel; Np = J.Np; Nc = J.Nc;
states = J.sizes.states; n_out = J.sizes.n_out; 
Est = J.state_est.Est;
con = J.constraints;

%**************** Title ****************%
disp('------------------------------------------------');
if(isa(J.StdModel,'jNL'))
    Model = J.StdModel.lin_model;
    n = 'NONLINEAR MODEL PREDICTIVE';
else
    n = 'LINEAR MODEL PREDICTIVE';
end
if(J.constraints.uncon)
    fprintf('UNCONSTRAINED %s CONTROLLER\n',n);
else
    fprintf('%s CONTROLLER\n',n);
end
%**************** Model ****************%
disp('------------------------------------------------');
disp('  Model Properties: ')
fprintf('States:                %2d\n',states);
fprintf('Manipulated Inputs:    %2d\n',J.sizes.nm_in);
if(J.sizes.nm_dist), fprintf('Measured Disturbances: %2d\n',J.sizes.nm_dist); end
% fprintf('Total Outputs:         %2d\n',J.sizes.n_out);
fprintf('Controlled Outputs:    %2d\n',J.sizes.nq_out);
if((n_out - J.sizes.nm_out) > 0), fprintf('Unmeasured Outputs:    %2d\n',n_out-J.sizes.nm_out); end
disp('------------------------------------------------');

%**************** Controller ****************%
disp('  Controller Specifications:')
if(J.mpcopts.Single)
    fprintf('Precision:             Single\n');
else
    fprintf('Precision:             Double\n');
end
fprintf('Sampling Period:       %1.2f sec\n',Model.Ts);
fprintf('Prediction Horizon:    %1d\n',Np);
if(length(Nc) > 1)
    fprintf(['Blocking Moves:        [',num2str(Nc),']\n']);
else
    fprintf('Control Horizon:       %1d\n',Nc);
end
if(~con.uncon)
    switch(lower(J.mpcopts.QPSolver))
        case 'quad_mehrotra',   fprintf('QP Solver:             jMPC Mehrotra''s Method [Double Precision]\n'); 
        case 'mquad_mehrotra',  fprintf('QP Solver:             jMPC Mehrotra''s Method (MEX) [Double Precision]\n'); 
        case 'mquad_mehrotramkl', fprintf('QP Solver:             jMPC Mehrotra''s Method (MEX, Intel MKL) [Double Precision]\n'); 
        case 'squad_mehrotra',  fprintf('QP Solver:             jMPC Mehrotra''s Method [Single Precision]\n'); 
        case 'msquad_mehrotra', fprintf('QP Solver:             jMPC Mehrotra''s Method (MEX) [Single Precision]\n'); 

        case 'quad_wright',   fprintf('QP Solver:             jMPC Wright''s Method [Double Precision]\n'); 
        case 'mquad_wright',  fprintf('QP Solver:             jMPC Wright''s Method (MEX) [Double Precision]\n'); 
        case 'mquad_wrightmkl', fprintf('QP Solver:             jMPC Wright''s Method (MEX, Intel MKL) [Double Precision]\n'); 
        case 'squad_wright',  fprintf('QP Solver:             jMPC Wright''s Method [Single Precision]\n'); 
        case 'msquad_wright', fprintf('QP Solver:             jMPC Wright''s Method (MEX) [Single Precision]\n'); 
        
        case 'quad_hildreth', fprintf('QP Solver:             jMPC Hildreth''s Method [Double Precision]\n'); 
        
        case 'quadprog', fprintf('QP Solver:             MATLAB''s quadprog [Double Precision]\n'); 
        case 'qpoases', fprintf('QP Solver:             qpOASES [Double Precision]\n'); 
        case 'qpip', fprintf('QP Solver:             QPC''s qpip (MEX) [Double Precision]\n'); 
        case 'qpas', fprintf('QP Solver:             QPC''s qpas (MEX) [Double Precision]\n'); 
        case 'clp', fprintf('QP Solver:             J. Forrest''s CLP (MEX) [Double Precision]\n'); 
        case 'ooqp', fprintf('QP Solver:             Wright & Gertz OOQP (MEX) [Double Precision]\n'); 
    end
    fprintf('QP Size:               %d Decision Vars, %d Constraints\n',length(J.QP.Hscale),length(J.QP.Ascale));
end
if(Est)
    disp('State Estimation:      YES')
else
    disp('State Estimation:      NO')
end
disp('------------------------------------------------');

%**************** Constraints ****************%
if(~con.uncon)
    u_op = J.lin.u_op; y_op = J.StdModel.C * J.lin.x_op; 
    disp('  Constraints:')
    if(con.ucon || con.delucon)
        con.u = con.u+u_op(J.index.iman_u)*[1 1 0];
        for i = 1:size(con.u,1)
            if(con.indices.iumin(i) || con.indices.iumax(i) || con.indices.idelu(i))
                fprintf('%5.2f <= IN%1d <= %2.2f,   %7.2f <= IN%1d/rate <= %2.2f\n',con.u(i,1),i,con.u(i,2),-con.u(i,3),i,con.u(i,3));
            end
        end
        disp(' ')
    end
    if(con.ycon)
        con.y = con.y+(y_op*[1 1]);        
        for i = 1:size(con.y,1)
            if(~isempty(con.soft.ind) && con.soft.ind(i))
                fprintf('%5.2f <= *OUT%1d <= %2.2f\n',con.y(i,1),i,con.y(i,2));
            elseif(con.indices.iymin(i) || con.indices.iymax(i))
                fprintf('%5.2f <= OUT%1d <= %2.2f\n',con.y(i,1),i,con.y(i,2));
            end
        end
    end
    if(isfield(con,'x'))
        disp(' ')
        for i = 1:size(con.x,1)
            fprintf('%5.2f <= X%1d <= %2.2f\n',con.x(i,1),i,con.x(i,2));
        end
    end
    disp('------------------------------------------------');
end

%**************** Warnings ****************%
if(isfield(con,'warning'))
    disp('  WARNINGS:')    
    for i = 1:length(con.warning)
        disp(con.warning{i})
    end
    disp('------------------------------------------------');
end

end