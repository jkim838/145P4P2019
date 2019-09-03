clear
clc

% define name of file to be opened
fname = '30fps-90sec-sample.json';
val = jsondecode(fileread(fname));

% preallocate arrays
YVelocities = zeros(1,1000);
IDReoccur = zeros(1,1000);
IDs = -1.*ones(1,1000);
classes = -1.*ones(1,1000);
changes = zeros(1,1000);
aggressives = zeros(1,1000);

% last array is the timestamp. Exclude as it has a different array size
lastFrame = length(val)-1;

% remove redundant top level
% top level was a cell array of structures which cant be plotted
S = [val{1:lastFrame,1}];

% define all base level variables into a simple matrix of entries for plotting
frame       = [S.Frame];
fps         = [S.FPS];
frameCount  = [S.FrameCount];
totalCount  = [S.TotalCount];

for i = 1:lastFrame
    % identify how many vehicles in particular frame
    frameVehicleLength = length(S(i).FrameVehicles);
    for j = 1:frameVehicleLength % load all second level information
        ID          = S(i).FrameVehicles(j).ID;
        class       = S(i).FrameVehicles(j).Class;
        yVel        = S(i).FrameVehicles(j).YVelocity;
        xVel        = S(i).FrameVehicles(j).XVelocity;
        Overspeed   = S(i).FrameVehicles(j).Overspeed;
        
        % MATLAB index starts from 1 therefore shift by one
        % vehicleID - 0 -> MATLAB index = 1)
        indexID = ID+1;
        
        % Record vehicleID
        IDs(indexID) = ID;
        
        % Record vehicle class
        if strcmp(class, 'car') class = 1; end
        if strcmp(class, 'bus') class = 2; end
        if strcmp(class, 'truck') class = 3; end
        classes(indexID) = class;
        
        % If errror in velocity is infinite (i.e. -1), then set to zero
        if yVel < 0
            yVel = 0;
        end
        
        % Accumulate vehicle speed for each unique vehicles
        YVelocities(indexID) = YVelocities(indexID) + yVel;
        IDReoccur(indexID) = IDReoccur(indexID)+1;
        
        % Load third level info on lane changing
        % Note still accessed via i as only one change variable per vehicle
        changeFlag      = S(i).FrameVehicles(j).LaneChange.change;
        aggressiveFlag   = S(i).FrameVehicles(j).LaneChange.aggressive;
        
        % If lane a specific ID lane changes in any frame then use the flag to record in matrix
        if changeFlag
            changes(indexID) = 1;
            changeFlag = 0;
        end
        % If a specific ID aggressively lane changes then flag and record to matrix
        if aggressiveFlag
            aggressives(indexID) = 1;
            aggressiveFlag = 0;
        end
        
    end
end

%% Plot average y-velocity and number of vehicles present in each frame
avgYVeloc(YVelocities,IDReoccur, IDs);

%% Plot the number of vehicles present in each frame
countPerFrame(frame, frameCount)

%% Plot of vehicle class types
classAndCount(classes, frame, totalCount)

%% Plot of lane changes
%NOTE: this function is an incomplete proof of concept for lane changing
%laneChanges(changes,IDs)