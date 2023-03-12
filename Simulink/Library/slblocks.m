function blkStruct = slblocks
%SLBLOCKS Define the Simulink library block representation.
%
%    Define the Simulink Library for jMPC Toolbox

%    Jonathan Currie 19/04/11
 
blkStruct.Name    = sprintf('jMPC\nToolbox');
blkStruct.OpenFcn = 'jMPClib';
blkStruct.MaskInitialization = '';

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it
Browser(1).Library = 'jMPClib';
Browser(1).Name    = 'JMPC Toolbox'; %must be capitals for alpha order (stink)

blkStruct.Browser = Browser;

% End of slblocks.m
