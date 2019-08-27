jsonData = jsondecode(fileread('30fps-90sec-sample.json'));

%Preallocate array size
%Initialize as zero array as values will be accumulated
YVelocities = zeros(1,1000);
IDOccCounter = zeros(1,1000);


%Preallocate array for IDs.
%Dummy value: -1
IDs = -1.*ones(1,1000);
FrameNo = -1*ones(1,1000);
FrameCountNo = -1*ones(1,1000);
%Iterate through all data
for i = 1:length(jsonData)-1
    realFrameID = jsonData{i}.Frame;
    frameIndex = realFrameID + 1;
    frameCount = jsonData{i}.FrameCount;
    FrameNo(frameIndex) = realFrameID;
    FrameCountNo(frameIndex) = frameCount;
    FrameVehicleLength = length(jsonData{i}.FrameVehicles);
    for j = 1:FrameVehicleLength
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
end

%Calculate average velocity of each unique vehicles
averageVelocities = YVelocities./IDOccCounter;
%Replace all infinite velocities (i.e. NaN) with -1, and remove those
%entries.
averageVelocities(isnan(averageVelocities)) = -1;
averageVelocities(averageVelocities==-1)=[];
FrameNo(FrameNo==-1)=[];
FrameCountNo(FrameCountNo==-1)=[];
%Remove all entries with dummy values (i.e. -1)
IDs(IDs==-1)=[];
%Combine ID array to information array
uniqueAvgVelocities = [IDs;averageVelocities];
CountNoPerFrame = [FrameNo;FrameCountNo];

figure(1);
subplot(2,1,1);
stem(uniqueAvgVelocities(2,1:length(uniqueAvgVelocities)));
title('Average Velocity of Each Vehicle');
xlabel('ID');
ylabel('Velocity (km/h)');
subplot(2,1,2);
bar(CountNoPerFrame(2,1:length(CountNoPerFrame)));
title('Number of Vehicles Present per Frame');
xlabel('Frame');
ylabel('Vehicle Count');