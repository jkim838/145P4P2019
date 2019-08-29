function laneChanges(changes,IDs)
% PROOF OF CONCEPT - shows we are tracking x-velocity information
% Identifies vehicles by ID that have potentially changed lanes

% Clean matrices and set changes to same length as IDs
IDs(IDs==-1)=[];
changes = changes(1, 1:length(IDs)); % not sure if this is correct

figure(5)
bar(IDs, changes);
title('ID of vehicles which have performed a lane change');
xlabel('IDs');
ylabel('Changed Lane');
end

