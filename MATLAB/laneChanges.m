function laneChanges(changes,IDs)
% BROKEN FUNCTION - does not match the the correct number of unique IDs
% hence was unable to plot changes vs frames

% Clean matrices and set changes to same length as IDs
IDs(IDs==-1)=[];
changes = changes(1, 1:length(IDs)); % not sure if this is correct

figure(5)
bar(IDs, changes);
xlabel('IDs');
ylabel('Changed Lane');
end

