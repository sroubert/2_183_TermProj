close all; clear all;

% Put names of all files in analysis folder into structure MATfiles
MATfiles = dir('analysis_1000/*.mat');

% Remove rows with file names that don't start with "OUTPUT"
deleteArray = [];
for i = 1:length(MATfiles)
    if ~contains(MATfiles(i).name, 'OUTPUT')
        deleteArray(end+1) = i;
    end
end
MATfiles(deleteArray) = [];

% Create table to hold outputs
analysisTypes = {'string','double','double','double','double','double',...
    'string','double','double','double'};
analysisNames = {'Simulation','Velocity','Angle','Spin','DOF','Fris Orientation',...
    'Obj Func','Vel Goal','Ang Goal','Spin Goal'};
outputTable = table('Size',[length(MATfiles), 10],...
            'VariableTypes',analysisTypes,'VariableNames',analysisNames);

% Put simulation outputs into table
for i = 1:length(MATfiles)
   name = MATfiles(i).name;
   load(strcat('analysis_1000/',name))
   name = convertCharsToStrings(name);
   
   outputTable{i,1} = name;
   outputTable{i,2} = simOutput(1);     % release velocity
   outputTable{i,3} = simOutput(2);     % release angle
   outputTable{i,4} = simOutput(3);     % release spin
   outputTable{i,5} = str2double(extractBetween(name,"OUTPUT","DOF"));  % DOF
   outputTable{i,6} = str2double(extractBetween(name,"frisOr","_obj")); % frisbee orientation
   outputTable{i,7} = extractBetween(name,"_obj","_vel");   % objective function
   outputTable{i,8} = str2double(extractBetween(name,"_vel","_ang"));   % velocity goal
   outputTable{i,9} = str2double(extractBetween(name,"_ang","_spin"));  % angle goal
   outputTable{i,10} = str2double(extractBetween(name,"_spin",".mat")); % spin goal
   
   
end

% Write table to excel file with current date & time in name
filename = strcat("analysis_1000/output_",string(datetime('now','Format','yyyy-MM-dd_HH-mm-ss')),".xlsx");
writetable(outputTable,filename)