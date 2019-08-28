function countPerFrame(frame, frameCount)

% Clean matrices. Remove all -1 dummy entries
frame(frame==-1)=[];
frameCount(frameCount==-1)=[];

% Combine Frame array and frameCount array
CountNoPerFrame = [frame;frameCount];

% Plotting
figure(2);
bar(CountNoPerFrame(2,1:length(CountNoPerFrame)));
title('Number of Vehicles Present per Frame');
xlabel('Frame');
ylabel('Vehicle Count');

end

