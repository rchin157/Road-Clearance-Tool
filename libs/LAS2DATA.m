%% CHANGELOG
%
% Uses matlas_tools to convert LAS files to useful data. All variables are
% extracted from the LAS file, but only the variables required by the user
% are set as output.
% --- Lloyd Karsten (Sept 13th, 2017)
%
% Edit: Added support for nested packaging
% By: Bruno Mello
% Data: 19 MAR 2019

%% FUNCTION
function [ hwdata,HEADER ] = LAS2DATA( filepath, varWanted )

% Nested packaging:
persistent import_cmd;
if(isempty(import_cmd))
    package_pattern_1 = '+.*\';
    package_pattern_2 = '+.*/';
    package_file_path = mfilename('fullpath');
    local_match = regexp(package_file_path, package_pattern_1, 'match');
    if(isempty(local_match))
        local_match = regexp(package_file_path, package_pattern_2, 'match');
    end
    cmd_str = '';
    if(~isempty(local_match))
        local_match = local_match{1};
        cmd_str = 'import ';
        curr_cmd = [];
        writing_cmd = false;
        for i=1:length(local_match)
            if(local_match(i) == '+')
                writing_cmd = true;
                continue;
            end
            if(writing_cmd)
                if(local_match(i) == '/' || local_match(i) == '\' || i == length(local_match))
                    cmd_str = [cmd_str, curr_cmd, '.'];
                    curr_cmd = [];
                    writing_cmd = false;
                else
                    curr_cmd = [curr_cmd,local_match(i)];
                end
                if(i == length(local_match))
                    cmd_str = [cmd_str, '*'];
                end
            end
        end
    end
    import_cmd = cmd_str;
end
eval(import_cmd);



%LAS2DATA
%
% LAS2DATA
% LAS2DATA( ... )
% [ ... ] = LAS2DATA( ... )
%
% Input:
%   varWanted - uses variable definition identical to readlas, where:
%           x - x location
%           y - y location
%           z - elevation information
%
%           i - intensity
%
%           r - number of this return
%           n - number of returns for given pulse
%           d - direction of scan flag
%           e - edge of flight line flag
%
%           c - classification
%           s - synthetic flag
%           k - key-point flag
%           w - withheld flag
%           
%           a - scan angle
%           u - user data
%           p - point source ID
%
%           t - time
%
%           R - red
%           G - green
%           B - blue
% 
% Output:
%   hwdata - variables selected by user are kept for output. If nothing is
%   selected, the typical output of 'txyzicap' is chosen

%% Variable Input Check
% check to ensure we have at least one input variable of 'char' type
if nargin < 1
    % need at least one input argument
    error('At least one input argument (character array) is needed');
elseif ~isa(filepath,'char')
    % input must be character array
    error('Filepath input must be a character array')
end

% check to ensure variable selection input exists and is correct
if ~exist('varWanted','var')
    varWanted = [];
end
% ...
% ...
% ...

%% Extract data from LAS file using matlas_tools
% input filename using '-i' preset for MEX file
parsestr = ['-i "' filepath '"'];
% collect header and data information from LAS file
[HEADER,DATA] = las2mat(parsestr);

%% Organize DATA output to single matrix
hwdata = [DATA.gps_time, DATA.x, DATA.y, DATA.z, double(DATA.intensity), double(DATA.classification), ...
    double(DATA.scan_angle_rank), double(DATA.point_source_ID)];

% adjust output to user variable specifications
if ~isempty(varWanted)
    columnsWanted = uservariables( varWanted, size(hwdata,2));
    hwdata = hwdata(:,columnsWanted);
end


%% FUNCTION CHECKS USER INPUT VARIABLE SPECIFICATIONS
function [ columnWanted ] = uservariables( varWanted, sizeHWDATA )
    % preallocate number of columns for deletion
    columnWanted = false(1,sizeHWDATA);
    
    % check for each input variable individually
    if contains(varWanted,'t')
        columnWanted(1,1) = true;
    end
    if contains(varWanted,'x')
        columnWanted(1,2) = true;
    end
    if contains(varWanted,'y')
        columnWanted(1,3) = true;
    end
    if contains(varWanted,'z')
        columnWanted(1,4) = true;
    end
    if contains(varWanted,'i')
        columnWanted(1,5) = true;
    end
    if contains(varWanted,'c')
        columnWanted(1,6) = true;
    end
    if contains(varWanted,'a')
        columnWanted(1,7) = true;
    end
    if contains(varWanted,'p')
        columnWanted(1,8) = true;
    end
end

end