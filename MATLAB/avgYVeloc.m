function avgYVeloc(YVelocities,IDOccCounter)
%AVGYVELOC Summary of this function goes here
%   Detailed explanation goes here

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

end

