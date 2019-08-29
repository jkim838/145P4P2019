function countPerFrame(frame, frameCount)
% This function generates a bar plot of the number of vehicls present in
% each frame. This can show the frequency and density of traffic flow

% Clean matrices. Remove all -1 dummy entries
frame(frame==-1)=[];
frameCount(frameCount==-1)=[];

% Combine Frame matrix and frameCount matrix
CountNoPerFrame = [frame;frameCount];

% Plotting
figure(2);
bar(CountNoPerFrame(2,1:length(CountNoPerFrame)));
title('Number of Vehicles Present per Frame');
xlabel('Frame');
ylabel('Vehicle Count');

end

