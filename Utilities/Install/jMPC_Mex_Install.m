%% Mex Install File

% There are a number of files which require compiling prior to being used
% with this toolbox. Below are the commands required to compile the
% relevant mex files.

%mex -setup

% Also ensure you are in the current directory of this file!

%Intel MKL (Optional, modify below function for your PC if required)
try
    [mkl_linkseq,~,~,~,~,mkl_ver] = jMPC_FindMKL('seq');
catch
    mkl_ver = '';
end

fprintf('\n------------------------------------------------\n');
fprintf('jMPC MEX FILE INSTALL\n\n');

%% mquad_wright.c
% Mex Version of the Wright QP Solver
clear mquad_wright
cd 'Solvers/Source';
try
    fprintf('Compiling Source: mquad_wright.c\n');
    mex -largeArrayDims mquad_x.c quad_wright.c jmath.c qp_mem.c qp_utils.c -IInclude -output mquad_wright
    fprintf('Build Complete\n\n');
    movefile(['mquad_wright.' mexext],'../../Solvers/','f') 
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';

%% msquad_wright.c
% Mex Version [Single Precision] of the Wright QP Solver
clear msquad_wright
cd 'Solvers/Source';
try
    fprintf('Compiling Source: msquad_wright.c\n');
    mex -largeArrayDims msquad_x.c quad_wright.c jmath.c qp_mem.c qp_utils.c -DSINGLE_PREC -IInclude -output msquad_wright
    fprintf('Build Complete\n\n');
    movefile(['msquad_wright.' mexext],'../../Solvers/','f') 
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';

%% mquad_mehrotra.c
% Mex Version of the Wright QP Solver
clear mquad_mehrotra
cd 'Solvers/Source';
try
    fprintf('Compiling Source: mquad_mehrotra.c\n');
    mex -largeArrayDims mquad_x.c quad_mehrotra.c jmath.c qp_mem.c qp_utils.c -DMEHROTRA -IInclude -output mquad_mehrotra
    fprintf('Build Complete\n\n');
    movefile(['mquad_mehrotra.' mexext],'../../Solvers/','f') 
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';


%% msquad_mehrotra.c
% Mex Version of the Wright QP Solver
clear msquad_mehrotra
cd 'Solvers/Source';
try
    fprintf('Compiling Source: msquad_mehrotra.c\n');
    mex -largeArrayDims msquad_x.c quad_mehrotra.c jmath.c qp_mem.c qp_utils.c -DSINGLE_PREC -DMEHROTRA -IInclude -output msquad_mehrotra
    fprintf('Build Complete\n\n');
    movefile(['msquad_mehrotra.' mexext],'../../Solvers/','f') 
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';


%% jMPC_sfcn.c
% C S Function MPC Controller
fprintf('Compiling Source: jMPC_sfcn.c\n');
clear jMPC_sfcn
cd 'Solvers/Source';
try
    eval('mex -largeArrayDims jMPC_sfcn.c quad_wright.c quad_mehrotra.c jmath.c qp_mem.c qp_utils.c mpc_utils.c -IInclude -DSFUN -DMEHROTRA');
    movefile(['jMPC_sfcn.' mexext],'../../Simulink/S Functions/','f')
    fprintf('Build Complete\n\n');
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';

%% jMPCEngine.c
% Mex Version of the jMPC Engine using Simple Linear Algebra
fprintf('Compiling Source: jMPCEngine.c\n');
clear jMPCEngine
cd 'Solvers/Source';   
try
    eval('mex -largeArrayDims jMPCEngine.c quad_wright.c quad_mehrotra.c jmath.c qp_mem.c qp_utils.c mpc_utils.c -llibut -IInclude -DMEX -DMEHROTRA');
    movefile(['jMPCEngine.' mexext],'../../Solvers/','f')
    fprintf('Build Complete\n\n');
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';


%% jMPCSEngine.c
% Mex Version of the jMPC Engine using Simple Linear Algebra [Single Precision]
fprintf('Compiling Source: jMPCSEngine.c\n');
clear jMPCSEngine
cd 'Solvers/Source';   
try
    eval('mex -largeArrayDims jMPCEngine.c quad_wright.c quad_mehrotra.c jmath.c qp_mem.c qp_utils.c mpc_utils.c -llibut -IInclude -output jMPCSEngine -DMEX -DMEHROTRA -DSINGLE_PREC');
    movefile(['jMPCSEngine.' mexext],'../../Solvers/','f')
    fprintf('Build Complete\n\n');
catch ME
    cd '../../';
    rethrow(ME);
end
cd '../../';

%% djacobi.c
fprintf('Compiling Source: djacobi.c\n');
if(~isempty(mkl_ver))
    clear djacobi
    cd 'Solvers/Source';    
    pre = 'mex -v -largeArrayDims djacobi.c';
    try
        eval([pre mkl_linkseq]);
        movefile(['djacobi.' mexext],'../../Solvers/','f')
        fprintf('Build Complete using MKL ver %s\n\n',mkl_ver);
    catch ME
        cd '../../';
        rethrow(ME);
    end
    cd '../../';
else
    fprintf('Intel MKL Could Not Be Found - File Skipped\n\n');
end

%%
fprintf('------------------------------------------------\n');

