close all; clear all;

% Put names of all files in analysis folder into structure MATfiles
MATfiles = dir('analysis/*.mat');

% Remove rows with file names that don't start with "OUTPUT"
deleteArray = [];
for i = 1:length(MATfiles)
    if ~contains(MATfiles(i).name, 'OUTPUT')
        deleteArray(end+1) = i;
    end
end
MATfiles(deleteArray) = [];

% Create table to hold outputs
analysisTypes = {'string','double','double','double'};
analysisNames = {'Simulation','Velocity', 'Angle', 'Spin'};
outputTable = table('Size',[length(MATfiles), 4],...
            'VariableTypes',analysisTypes,'VariableNames',analysisNames);

% Put simulation outputs into table
for i = 1:length(MATfiles)
   name = MATfiles(i).name;
   load(strcat('analysis/',name))
   name = convertCharsToStrings(name);
   
   outputTable{i,1} = name;
   outputTable{i,2} = simOutput(1);
   outputTable{i,3} = simOutput(2);
   outputTable{i,4} = simOutput(3);
end

% Write table to excel file with current date & time in name
filename = strcat("analysis/output_",string(datetime('now','Format','yyyy-MM-dd_HH-mm-ss')),".xlsx");
writetable(outputTable,filename)