clear
clc

% define name of file to be opened
fname = '30fps-90sec-sample.json';
val = jsondecode(fileread(fname));

% preallocate arrays
YVelocities = zeros(1,1000);
IDReoccur = zeros(1,1000);

% last array is the timestamp. Exclude as it has a different array size
lastFrame = length(val)-1;

% remove redundant top level
% was a cell array of structures which cant be plotted
S = [val{1:lastFrame,1}];

% define all base level variables into a simple matrix of entries for plotting
frame       = [S.Frame];
fps         = [S.FPS];
frameCount  = [S.FrameCount];
totalCount  = [S.TotalCount];

for i = 1:lastFrame
    % identify how many vehicles in particular frame
    frameVehicleLength = length(S(i).FrameVehicles);
    for j = 1:frameVehicleLength
        ID          = S(i).FrameVehicles(j).ID;
        class       = S(i).FrameVehicles(j).Class;
        yVel        = S(i).FrameVehicles(j).YVelocity;
        xVel        = S(i).FrameVehicles(j).XVelocity;
        Overspeed   = S(i).FrameVehicles(j).Overspeed;
        
        %If errror in velocity is infinite (i.e. -1), then set to zero
        if yVel < 0
            yVel = 0;
        end
        %MATLAB index starts from 1 therefore shift by one
        %vehicleID - 0 -> MATLAB index = 1)
        indexID = ID+1;
        
        %Record vehicleID
        IDs(indexID) = ID;
        %Accumulate vehicle speed for each unique vehicles
        YVelocities(indexID) = YVelocities(indexID) + yVel;
        IDReoccur(indexID) = IDReoccur(indexID)+1;
    end
end

%% can use the following to test data is working
% plot frame number vs total vehicle count

% figure
% plot(frame, totalCount)
% title('Frame Number vs Total Number of Vehicles Counted')
% xlabel('Frame Number')
% ylabel('Total Vehicles Counted')


