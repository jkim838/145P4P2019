function  classAndCount(classes, frame, totalCount)
% This function generates two plots. The first is a pie chart showing the
% percentage of vehicles of each type in the video feed. The secon is a
% plot of the the accumulative count of total vehicles in each frame

% Clean matrices. Remove all -1 dummy entries
frame(frame==-1)=[];
classes(classes==-1)=[];

% count the occurence of each type
counts = histc(classes, unique(classes));

% Plotting
% NOTE: a warning will be displayed in command window if no vehicles of a
% type present in data
figure(3)
pie(counts);
labels = {'Car','Bus','Truck'};
legend(labels,'Location','southoutside','Orientation','horizontal','fontsize',22);
title('Percentage of vehicles of each type','fontsize',24)


figure(4)
plot(frame, totalCount); % could split class data to have a line for each
title('Cumulative Number of Vehicles Counted Over Time')
xlabel('Frame Number')
ylabel('Total Vehicles Counted')
end

