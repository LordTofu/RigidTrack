%% Import data from text file.
% Script for importing data from the following text file:
%
%    C:\Users\Testrechner\Documents\5TOL_IP_2016_Development\4000_Visualization\4100_Benchmark_Visualization\Camera_Tracking\RigidTrack\RigidTrack\logData.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2016/12/23 15:32:40

%% Initialize variables.
filename = 'logData.txt'
delimiter = ';';

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
t = dataArray{:, 1};
x = dataArray{:, 2};
y = dataArray{:, 3};
z = dataArray{:, 4};
euler1 = dataArray{:, 5};
euler2 = dataArray{:, 6};
euler3 = dataArray{:, 7};
velx = dataArray{:, 8};
vely = dataArray{:, 9};
velz = dataArray{:, 10};


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;