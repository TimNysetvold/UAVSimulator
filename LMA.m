function objective = LMA(j,dronearray,conflictindex,ownobjective)
    
    ownship=dronearray(j,:);
    ownobjective=ownobjective{j,1};
    
    allconflictdistances=zeros(size(conflictindex,1),1);
    
    for i=1:size(conflictindex(:,1))
        curconflictdrone=dronearray(conflictindex(i),:);
        dist=pdist([ownship(1:2);curconflictdrone(1:2)]);
        allconflictdistances(i,:)=dist;
    end
    [~,index]=min(allconflictdistances);
    trueindex=conflictindex(index(1),:);
    conflictdrone=dronearray(trueindex(1),:);

    objective=[conflictdrone(1:2)';-1];
    objective=[checkObjectiveColinearity(ownship,ownobjective,objective',-1)';-1]

end