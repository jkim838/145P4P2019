clear
clc

% define name of file to be opened
fname = 'Mon Aug 26 20_31_41 2019-30fps-90sec-sample.json';
val = jsondecode(fileread(fname));

% last array is the timestamp. Exclude as it has a different array size
lastFrame = length(val)-1;

% remove redundant top level
% was a cell array of structures which cant be plotted
S = [val{1:lastFrame,1}];

% define all the variables into a simple matrix of entries for plotting
frame       = str2double({S.Frame});
fps         = str2double({S.FPS});
framecount  = str2double({S.FrameCount});
totalCount  = str2double({S.TotalCount});

% plot frame number vs total vehicle count
figure
plot(frame, totalCount)
title('Frame Number vs Total Number of Vehicles Counted')
xlabel('Frame Number')
ylabel('Total Vehicles Counted')

% each frame is unique in size so call it as required
% eg. S(1).FrameVehicles.ID
%TODO find out how to convert structure within structure

for i = 1:lastFrame
    ID = S.FrameVehicles(i).ID;
    
    realID = jsonData{i}.FrameVehicles(j).ID;
    realYVel = jsonData{i}.FrameVehicles(j).YVelocity;
    %If velocity is infinite (i.e. -1), then replace with zero
    if realYVel < 0
        realYVel = 0;
    end
    %Because MATLAB indexing starts from 1, all IDs will have its index
    %shifted by one. (e.g. vehicleID = 0 -> MATLAB index = 1)
    indexID = realID+1;
    %Record vehicleID
    IDs(indexID) = realID;
    %Accumulate vehicle speed for each unique vehicles
    YVelocities(indexID) = YVelocities(indexID) + realYVel;
    IDOccCounter(indexID) = IDOccCounter(indexID)+1;
    
end
