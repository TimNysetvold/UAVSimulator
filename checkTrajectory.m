%checkTrajectory method that takes in conflicts and checks if they're
%actually threats

function [conflictsIndices] = checkTrajectory(ownIndex, droneArray, conflictsIndicesIn, shipObjectives, collisiondistance)

    %get xyz of ownship current location and objective
    ownship=droneArray(ownIndex,:);
    ownshipObjective = shipObjectives{ownIndex,1};
    
    %Grab the right x y z from current location and objective
    ownshipObjective = ownshipObjective([1 2 3]);
    ownship = ownship([ 1 2 3]);
    
    indicesToRemove = [];
    
    for conflictIndex=1:size(conflictsIndicesIn,1)
        %realConflictIndex = conflictsIndicesIn(conflictIndex);
        
        %get xyz of conflict current location and objective
        conflict = droneArray(conflictsIndicesIn(conflictIndex),:);
        conflictObjective = shipObjectives{conflictIndex,1};
        
        
        %Grab the right x y z from current location and objective
        conflictObjective = conflictObjective([ 1 2 3]);
        conflict = conflict([ 1 2 3]);
        
        minimumDistance = computeDistanceBetween3DLines(ownship, ownshipObjective, conflict, conflictObjective); 
        
        %this value will need to be changed to the crash distance
        if minimumDistance > collisiondistance
            %get rid of the conflict as it's not going to be a problem.
            %add to indices to remove
            indicesToRemove = [indicesToRemove conflictIndex];
            %conflictsIndicesIn(conflictIndex) = [];
        end
        
    end

    
    %just loop backwards man.
    %for loop through indicesToRemove backward
    %remove that index from indicesIn
    while indicesToRemove > 0
        
        indexToRemove = indicesToRemove(end);
        
        conflictsIndicesIn(indexToRemove) = [];
        
        %remove last element of indicesToRemove
        indicesToRemove(end) = [];
        %indicesToRemove(size(indicesToRemove)) = [];
    end
    
    %return the updated array at the end
    conflictsIndices = conflictsIndicesIn;