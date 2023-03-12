function SaveWMV(filename,data,varargin)
% Save captured simulation video to WMV file
%
%   SaveWMV(filename,data)
%   SaveWMV(filename,data,framerate)

if(ndims(data) < 4)
    error('Your video data must be 4-D');
end
if(~strcmp(class(data),'uint8'))
    error('Your video data must be a uint8');
end
if(isempty(varargin)); fR = 30; else fR = varargin{1}; end
if(fR > 100 || fR < 1)
    error('Your framerate must be between 1 and 100 fps');
end

hmfw = video.MultimediaFileWriter([filename '.wmv'],'AudioInputPort',false,'VideoInputPort',true,'FileFormat','WMV');
set(hmfw,'FrameRate',fR);

len = size(data,4);
h = waitbar(0,'Writing WMV File');

for i = 1:len;
    step(hmfw,data(:,:,:,i));
    waitbar(i/len,h);
end

release(hmfw);
close(h);
