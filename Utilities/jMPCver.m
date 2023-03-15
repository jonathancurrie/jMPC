function current_version = jMPCver
%jMPC Version Information
%  Return jMPC software version. This document also contains version update
%  information.

current_version = 3.22;

%v3.22 (15/03/23)
% - Updated for Control Engineering reference
% - Rebuilt all MEX files
% - Lots of little fixes and updates

%v3.21 (25/09/14)
% - Corrected installer to check for 2012a or later
% - Regenerated MPC test results
% - Fixed Simulink bus/mux bug for 2013b or later
% - Fixed bug in Simulink MPC block with uncon logical

%v3.20 (14/12/13)
% - Fixed convergence bug in C-code quad_wright solvers
% - Substantial updates to quad_wright algorithm (performance + memory + robustness)
% - Fixed quad_mehrotra
% - Fixed bug with soft constraints on not fully constrained MIMO systems
% - Added single precision versions of quad_wright, quad_mehrotra, and MPC simulations
% - Lots of new options and performance improvements

%v3.16 (11/07/13)
% - Update to use mklJac for linearization if available
% - Fixed bug in Simulink MPC block with vector setpoint entries
% - Removed C-code source for MPC, moved to commercial license
% - Rebuilt MEX files with R2013a, MKL v11 R5 and VC++ 2012

%v3.15 (24/12/12)
% - Fixed bug in nonlinear simulations in GUI
% - Updates to embed(), moved to commercial license
% - Minor documentation updates
% - Rebuilt MEX files with R2012a and MKL v11 R1

% v3.12 (04/02/12)
% - Rebuilt MEX files with R2011b
% - Added pre-requisite checking to installer
% - Fixed bug with HELP location
% - Toolbox released under the BSD 3 Clause License

% v3.11 (05/07/11)
% - Fixed bug with look ahead enabled in Simulink
% - Added ability to specify hard or soft constraints on each output

% v3.10 (28/05/11)
% - Fixed GUI Save / Load Problem with Soft Constraints
% - Fixed linearization error
% - Fixed bug with Simulink simulations using wrong workspace
% - Added ODE Plant for jNL Simulink Simulations
% - Added measured disturbances to MPC problem
% - Added two new nonlinear examples
% - Added ability to name inputs and outputs for plots
% - Added PDF Documentation
% - Added 3D Simulink MPC Demos
% - Added S function to include VR World + Simulink Graphs
% - Added jMPCset for setting up advanced controller options
% - Changed simulations to only require T samples (instead of T+1)
% - Changed jMPC Simulink Library setup + added new pictures
% - Changed Raw Simulink MPC blocks to avoid using DSP Blocks
% - Overhauled plotting MPC results
% - linearize(jNL) is now much more robust plus added selectable solvers
% - Substantial html documentation update

% v3.00 (06/04/11)
% - Added Blocking to Control Problem
% - Fixed MPC Toolbox MIMO system problems
% - Added Soft Output Constraints to Control Problem
% - Updated GUI with Blocking + Soft Y
% - Updated S Function with Blocking + Soft Y
% - Added 'MEX' option for rapid MPC simulation (jMPCEngine.c)

% v2.66 (21/02/2011)
% - Updates to GUI
% - Added ability to interface to MATLAB MPC Toolbox

% v2.65 (23/11/2010)
% - Update to use MKL 10.3 Libraries
% - Fixed library link for MKL 10.2

% v2.64 (28/10/2010)
% - Update to TabPanel 2.8.1 
% - Option to automatically remove older versions added to install
% - Renaming of install files to avoid conflicts

% v2.63 (24/10/2010)
% - Bug fix to MEX Install not finding simstruc.h due to mex.pl change in
%   R2010b

% v2.62 (20/10/2010)
% - Bug fix to GUI not displaying continuous LTI plants
% - Bug fix to jSS Class not finding R2010b control toolbox

% v2.61 (29/7/2010)
% - Bug fix to Install Test not finding Simulink Path
% - Bug fix to GUI written for R2010a only
% - Rebuilt mex files using VC++ 2010

% v2.6 (20/7/2010)
% - Substantial HTML Docuemntation update
% - Added 3 examples to documentation
% - Added demo mode to GUI

% v2.5 (15/12/2009)
% - Added clock(jMPCobj) to estimate implementation timing
% - Added linearize(jNLobj) to automatically linearize a set of non linear
% odes
% - Added operating point properties to jSS class
% - MPC builder automatically detects linearized models and applies biases
% - Added ability to specify an output weight as 0, thereby making the
% corresponding output 'uncontrolled', and reducing the QP size
% - Updated C S function to reflect above changes (beta)
% - HTML Documentation substantially updated

% v2.4 (28/11/2009)
% - Formalized source code directory (under development)'
% - Separated quad_wright.c & quad_wright_blas.c and added as includes
% - Added embed(jMPCobj) to auto generate MPC implementations (under
% development)

% v2.3 (15/11/2009)
% - Added C S function to Simulink Library
% - Added this versioning system
% - HTML Documentation updated

% v2.2 (22/10/2009)
% - jMPC HTML Documentation added (under development)
% - Added help to major functions

% v2.1 (9/10/2009)
% - Class names shortened
% - Code commented and tidied

% v2.0 (8/10/2009)
% - Added dead time integration
% - Added Non Linear Objects (jNL)
% - Added Lund QP Solvers & Examples

% v1.9 (20/9/2009)
% - Files collected as a distributable zip folder

% v1.8 (16/9/2009)
% - GUI html help added

% v1.7 (14/9/2009)
% - major GUI changes

% v1.6 (11/9/2009)
% - jGUI object added

% v1.5 (10/9/2009)
% - GUI added
% - ScrollPlot object added

% v1.4
% - Major implementation changes

% v1.3 
% - Simulink MPC written

% v1.2 
% - Major implementation changes

% v1.1 
% - Re written as an object orientated system

% v1.0 (15/6/2009)
% - Initial jMPC idea, implemented as a collection of m files


end

