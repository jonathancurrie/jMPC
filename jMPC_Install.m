function jMPC_Install
%% Installation File for jMPC

% In order to run this tool, please run this file to setup the required
% directories. You MUST be in the current directory of this file!

cpath = cd;

try
    cd('Utilities');
catch %#ok<CTCH>
    error('You don''t appear to be in the jMPC Toolbox directory');
end
cver = jMPCver();

fprintf('\n------------------------------------------------\n')
fprintf(['  INSTALLING jMPC TOOLBOX ver ' sprintf('%1.2f',cver) '\n\n'])

cd(cpath);

%Perform pre-req check
if(~preReqChecks(cpath))
    return;
end

fprintf('\n- Checking for previous versions of jMPC Toolbox...\n');
no = jMPC_Uninstall('jMPC_Install.m',0);
if(no < 1)
    fprintf('Could not find a previous installation of jMPC Toolbox\n');
else
    fprintf('Successfully uninstalled previous version(s) of jMPC Toolbox\n');
end

fprintf('\n- Adding jMPC Paths to MATLAB Search Path...');
genp = genpath(cd);
genp = regexp(genp,';','split');
%Folders to exclude from adding to Matlab path
remInd1 = strfind(genp,'vti_cnf');
remInd2 = strfind(genp,'vti_pvt');
ind = [];
%Can't seem a way to check if cells are empty as a vector?
for i = 1:length(remInd1)
    if(any(remInd1{i}) || any(remInd2{i}))
        ind = [ind i];  %#ok<AGROW>
    end 
end
%Remove paths from above and add to matlab path
genp(ind) = [];
addpath(genp{:});

rehash
fprintf('Done\n\n');

in = input('- Would You Like To Save the Path Changes? (Recommended) (y/n): ','s');
if(strcmpi(in,'y'))
    try
        savepath;
    catch
        warning(['It appears you do not have administrator rights on your computer to save the Matlab path. '...
                 'In order to run jMPC Toolbox you will need to install it each time you wish to use it. To fix '...
                 'this please contact your system administrator to obtain administrator rights.']);
    end
end
fprintf('\n');

in = input('- Would You Like To Run Post Installation Tests? (Recommended) (y/n): ','s');
if(strcmpi(in,'y'))
    jMPC_Install_Test(1);
end
fprintf('\n');

fprintf('\njMPC Toolbox Installation Complete!\n');
disp('------------------------------------------------')


function no = jMPC_Uninstall(token,del)
no = 0;
%Check nargin in, default don't delete and jmpc mode
if(nargin < 2 || isempty(del))
    del = 0;
end

%Check if we have anything to remove
paths = which(token,'-all');
len = length(paths);
%If mode is opti, should always be at least 1 if we are in correct directory
if(~len)
    error('Expected to find "%s" in the current directory - please ensure you are in the jMPC Toolbox directory',token);        
%If mode is opti, and there is one entry    
elseif(len == 1)
    %if len == 1, either we are in the correct folder with nothing to remove, or we are in the
    %wrong folder and there are files to remove, check CD
    if(any(strfind(paths{1},cd)))
        no = 0;
        return;
    else
        error('Expected to find "%s" in the current directory - please ensure you are in the jMPC Toolbox directory',token);
    end    
else %old ones to remove
    %Remove each folder found, and all subdirs under
    for n = 2:len
        %Absolute path to remove
        removeP = paths{n};
        %Search backwards for first file separator (we don't want the filename)
        for j = length(removeP):-1:1
            if(removeP(j) == filesep)
                break;
            end
        end
        removeP = removeP(1:max(j-1,1));        

        %Everything is lowercase to aid matching
        lrpath = lower(removeP);
        opath = regexp(lower(path),';','split');

        %Find & Remove Matching Paths
        no = 0;
        for i = 1:length(opath)
            %If we find it in the current path string, remove it
            fnd = strfind(opath{i},lrpath);        
            if(~isempty(fnd))  
                rmpath(opath{i});
                no = no + 1;
            end
        end
        
        %Check we aren't removing our development version
        rehash;
        if(isdir([removeP filesep 'Testing'])) %is this robust enough?
            fprintf('Found development version in "%s", skipping.\n',removeP);
            return;
        end

        %If delete is specified, also delete the directory
        if(del)
            stat = recycle; recycle('on'); %turn on recycling
            rmdir(removeP,'s'); %not sure if we dont have permissions here
            recycle(stat); %restore to original
        end
    end    
end


function OK = preReqChecks(cpath)
%Search for each required prereq
% Note we no longer search the registry, simply check if we can load a mex
% file which requires each runtime

if(~isempty(strfind(computer,'64')))
    arch = 'x64';
else
    arch = 'x86';
end

mver = ver('MATLAB');
fprintf('- Checking MATLAB version and operating system...\n');
vv = regexp(mver.Version,'\.','split');
if(str2double(vv{1}) <= 9)
    if(str2double(vv{2}) <= 4)
        fprintf(2,'jMPC is designed for MATLAB 2018b or above.\nIt will install into lower versions, but you may experience reliability problems.\nPlease upgrade to R2018b or later.\n');
    end
end
switch(mexext)
    case 'mexw32'
        warning('jMPC Toolbox contains MEX files compiled for 64bit Windows PCs only. Only MATLAB MPC simulations will be available.');
        OK = 1; return;
    case 'mexw64'
        fprintf('MATLAB %s 64bit (Windows x64) detected\n',mver.Release);
    otherwise      
        warning('jMPC Toolbox contains MEX files compiled for Windows PCs only. Only MATLAB MPC simulations will be available.');
        OK = 1; return;
end

fprintf('\n- Checking for required pre-requisites...\n');

havVC = true;
missing = false;

%Check for VC++ 2019
cd 'Solvers'
try
    a = mquad_wright; %#ok<NASGU>
catch
    havVC = false;
end
cd ../
cd(cpath);
%See if missing anything
if(~havVC)
    missing = true;
end

%Print Missing PreReqs
if(~havVC)
    fprintf(2,'Cannot find the Microsoft VC++ 2019 %s Redistributable!\n',arch); 
else
    fprintf('Found the Microsoft VC++ 2019 %s Redistributable\n',arch); 
end

%Install Instructions for each Package
if(missing)
    fprintf(2,'\nYou are missing one or more pre-requisites. Please read the instructions below carefully to install them:\n\n');
    
    if(~havVC)
        fprintf(2,' Microsoft VC++ 2019:\n  - Download from: https://aka.ms/vs/17/release/vc_redist.x64.exe\n');
        fprintf(2,'  - When prompted, select the ''%s'' package. Once downloaded, install it.\n\n',arch);
    end
    
    fprintf(2,'\nOnce you have downloaded AND installed all the above packages, you MUST restart MATLAB.\n\nIf this message appears again after installing the above packages, try restarting your computer.\n\n\n');
    
    OK = false;
else
    OK = true;
end
