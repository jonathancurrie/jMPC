function J = single(J)
%SINGLE  Convert the jMPC Object to Single Precision

%Convert Models
J.Model = single(J.Model);
J.StdModel = single(J.StdModel);
%Convert Horizons
J.Np = single(J.Np);
J.Nc = single(J.Nc);
J.Nb = single(J.Nb);
%Convert Structures
J.QP = struct2single(J.QP);
J.constraints = struct2single(J.constraints);
J.constraints.soft = struct2single(J.constraints.soft);
J.pred = struct2single(J.pred);
J.state_est = struct2single(J.state_est);
J.initial = struct2single(J.initial);
J.lin = struct2single(J.lin);
J.index = struct2single(J.index);
J.sizes = struct2single(J.sizes);
J.mpcopts = struct2single(J.mpcopts);
J.mpcopts.Single = 1;


