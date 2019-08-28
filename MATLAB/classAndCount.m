function  classAndCount(classes, frame, totalCount)

% Clean matrices. Remove all -1 dummy entries
frame(frame==-1)=[];
classes(classes==-1)=[];

% count the occurence of each type
counts = histc(classes, unique(classes));

% Plotting
figure(3)
subplot(2,1,1);
% witcraft required to label pie charts it MATLAB
chart = pie(counts);
T = chart(strcmpi(get(chart,'Type'),'text'));
P = cell2mat(get(T,'Position'));
set(T,{'Position'},num2cell(P*0.4,2))
text(P(:,1),P(:,2),{'Car','Bus','Truck'})

subplot(2,1,2);
plot(frame, totalCount); % could split class data to have a line for each
title('Frame Number vs Total Number of Vehicles Counted')
xlabel('Frame Number')
ylabel('Total Vehicles Counted')
end

