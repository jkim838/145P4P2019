function avgYVeloc(YVelocities,IDReoccur, IDs)
% This function generates a stem plot of the vehicles based on their IDs 
% and average velocity

% Calculate average velocity of each unique vehicles
averageVelocities = YVelocities./IDReoccur;

% Clean matrices. Remove all infinite values or -1 dummy entries
averageVelocities(isnan(averageVelocities)) =[];
IDs(IDs==-1)=[];

% Combine ID array to information array
uniqueAvgVelocities = [IDs;averageVelocities];

% Plotting
figure(1)
stem(uniqueAvgVelocities(2,1:length(uniqueAvgVelocities)));
title('Average Velocity of Each Vehicle');
xlabel('ID');
ylabel('Velocity (km/h)');

end

