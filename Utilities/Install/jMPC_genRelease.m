function jMPC_genRelease()
% Generate a Release for jMPC Toolbox

% J.Currie December 13

%Local
% 0) Recompile MEX files
% 1) Update jmpcver version, release notes and date
% 2) Update documentation release notes and date
% 3) Remove any testing files!

%Website
% 1) Generate new packages and upload
% 2) Update php download info page for new versions
% 3) Update jMPC page with new features
% 4) Update homepage with new version

skipDir = 'Source';
skipFiles = {'jMPC_Fx.dwt','jMPC_Normal.dwt','jMPC_Sim.dwt','jMPC_TopPage.dwt','NLupdate.m','simNLJMPC.m'};
pcodeFiles = {'embedJMPC'};
contentsLoc = 'Utilities/jMPC';

buildTbxRelease('jMPC',jMPCver,cd,skipDir,skipFiles,pcodeFiles,contentsLoc);




