 function numcrashes = main (input1,input2,input3)
%This is the base code for the drone
%simulator for use by Dr. John Salmon.
%Written by Tim Nysetvold.

close all

%RNG data. This gives each simulation a different seed,
%which is recorded so that the sim can be replayed if necessary.
rng('shuffle');
scurr=rng;

%% Basic parameters for simulation.
% 1 iteration is equal to 0.1 second of flight time in simulation.
iterations=288000+9000; %5 hours, plus the required 15 minutes (9000
% iterations) of warm-up time.
% iterations=9000; should be sufficenit
collisiondistance=.15;
% the FAA has declared the "near miss" distance for drones to be 500 ft
% (150 meters). This would have a value of .15 in this sim.
metalcollisiondistance=.015;
% 150m will probably not result in real collisions between drones.
% We use this metric to determine when a metal-on-metal crash would actually be likely.
baselength=10;
altlength=1;
layers=input3;
% The base length determines the size of the city the drones inhabit. It is
% given in kilometers. 

%% Instantiation of drone vectors.
% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

% Current types available: 1=police; 2=AmazonAir; 3=Hobbyist; 4=Military
% Remember, priority 1 is the highest priority; priority infinity would be
% the lowest.

% droneset 1
numdrones(1)=input1; %how many UAVs
cruiseheight(1)=.55;
xbase(1)=baselength/2; %the home base x position of these UAVs
ybase(1)=baselength/2; %the home base y position of these UAVs
dronemaxvel(1)=.003; %this is equal to 30 m/(tick). This value must be input in kilometers per tick
dronemaxaccel(1)=.0005; %the maximum acceleration available to this UAV. Given in km/(tick^2), which kind of sucks
separation(1)=input2; %the separation standard this UAV attempts to maintain
dronestamina(1)=1000000; %the battery or fuel life of these UAVs
priority(1)=2; %How much responsibility this UAV has for avoidance. 1 is least, but only matters in comparison.
type(1)=2;
pause(1)=0; %UAVs do not start out paused

%droneset 2
numdrones(2)=0;
cruiseheight(2)=.55;
xbase(2)=baselength/2;
ybase(2)=baselength/2;
dronemaxvel(2)=.003;
dronemaxaccel(2)=.0005;
separation(2)=input2;
dronestamina(2)=1000000;
priority(2)=3;
type(2)=2;
pause(2)=0;

%droneset 3
% numdrones(3)=0;
% cruiseheight(3)=.5;
% xbase(3)=baselength/2;
% ybase(3)=baselength/2;
% dronemaxvel(3)=.003;
% dronemaxaccel(3)=input2;
% separation(3)=input3;
% dronestamina3=100000;
% type(3)=2;
% priority(3)=3;
% pause(3)=0;  

loiter=0; %Currently unused for all drones


%% Instatiation of things that we're recording
mindist=zeros(iterations,3); %the smallest distance between drones for each iteration
activedrones=zeros(iterations,1); %how many drones are actually flying per iteration
metalcrashes=[]; %Vector that will store locations and drone identities of crashes
conflictsrecord=0; %how many conflicts have occurred
faacrashes=0; %how many NMACs under current FAA definition
global objectivesreached;
objectivesreached=0; %how many goals have been accomplished
% This creates objective and drone arrays for use in the main loop. Using multiple
% small arrays allow for multiple types of drones to co-exist easily.
% The matrix totaldronearray(num) will have a number
% of drones numdrones, created with parameters
% input at initiation. Saves original drones in 
% stats for data lookup if necessary.
dronestats=zeros(length(numdrones),18);
totaldronearray=zeros(sum(numdrones),18);
objectivearray=[];

%% Debugging code; used to keep track of a few drones
linecapture.x=[];
linecapture.y=[];
linecapture.z=[];
 


%% This section initializes the totaldronearray and the objective array.
%%Because we do not know how many sets of drones we have, we use this loop
%%structure to find out and concatenate them.
for dronenum=1:length(numdrones)
    if dronenum==1
        start=1;
    else
        start=sum(numdrones(1:(dronenum-1)))+1;
    end
    totaldronearray(start:sum(numdrones(1:dronenum)),:)=newDroneArray(numdrones(dronenum),dronemaxvel(dronenum),dronemaxaccel(dronenum),separation(dronenum),priority(dronenum),type(dronenum),dronestamina(dronenum),xbase(dronenum),ybase(dronenum),pause(dronenum),baselength,loiter,cruiseheight(dronenum));
    dronestats(dronenum,:)=newDroneArray(1,dronemaxvel(dronenum),dronemaxaccel(dronenum),separation(dronenum),priority(dronenum),type(dronenum),dronestamina(dronenum),xbase(dronenum),ybase(dronenum),pause(dronenum),baselength,loiter,cruiseheight(dronenum));
end

for dronenum=1:length(numdrones)
    if dronenum==1
        start=1;
    else
        start=sum(numdrones(1:(dronenum-1)))+1;
    end
    objectivearray=[objectivearray;newObjectiveArray(totaldronearray(start:sum(numdrones(1:dronenum)),:),baselength)];
end

%% Debug code forcing certain drones to go to certain spots
% [totaldronearray(1:4,1:3),objectivearray(1:4,1:3)]=debugScenario1();
% [totaldronearray(1:4,1:3),objectivearray(1:4,1:3)]=debugScenario2();
% [totaldronearray(1,15)]=debugScenario3();
% stopper=250;

%% Converts beginning objectives into the patharray, which may become ragged
%as time progresses. Generalized coordination of arrays into usable form
%happens here.

patharray=num2cell(objectivearray,2);

%Combines arrays for ease of in-loop processing.
%patharray=num2cell(objectivearray); 
intermediatearray0s=zeros(numdrones(1)+numdrones(2),4); %This array has any unexpected deconfliction waypoints along the path.
layerMat=layerCreator(altlength, layers);

for dronenum=1:size(totaldronearray,1)
    totaldronearray(dronenum,3)=AltDetLayers([objectivearray(dronenum,1:2),totaldronearray(dronenum,3)],totaldronearray(dronenum,1:3),layerMat);
    totaldronearray(dronenum,12)=totaldronearray(dronenum,3);
    patharray{dronenum}(1,3)=totaldronearray(dronenum,3);
end
markeddrones=1:2:(input1);
%markeddrones=[markeddrones,(input1*2):(input1*2)+3];
%markeddrones=1:11;

%% Begin main control loop
for currentIteration=1:iterations
    %This is the main control loop for the simulation. Each pass through
    %the loop is one tick.
        
    intermediatearray=intermediatearray0s;
    %We reset the list of intermediate steps.
   
    distances=analyzeDistances(totaldronearray);
    %%%Start Possible new Version
    %[distances_xyz,distances_xy,distances_z] = analyzeDistances(totaldronearray);
    %%%End Possible new Version
    [totaldronearray,newcrashes]=findMetalCrashes(distances,metalcollisiondistance,totaldronearray,currentIteration,baselength);
    metalcrashes=[metalcrashes;newcrashes];
    [newcrashes]=findFAACrashes(distances,faacrashes,collisiondistance,totaldronearray,currentIteration,baselength);
    faacrashes=faacrashes+size(newcrashes,1);
    
    %This section determines crashes that have occurred.
    conflictindex=findConflicts(distances,totaldronearray);
    conflictsrecord=conflictsrecord+size(conflictindex,1);
    %determine conflicts- if a drone is in the bubble of an equal or higher
    %priority drone, return that drone's index and the index of its
    %superior
    
    %% Graphing section
%     linecapture.x=[linecapture.x,totaldronearray(markeddrones,1)];
%     linecapture.y=[linecapture.y,totaldronearray(markeddrones,2)];
%     linecapture.z=[linecapture.z,totaldronearray(markeddrones,3)];
%     graphDrones(totaldronearray,patharray,newcrashes,linecapture,baselength)

    %% In this section, each drone moves.
    for dronenum=1:sum(numdrones)
       if(~isempty(conflictindex)) %if there is a conflict, find the entry to which it corresponds
           [row,~]=find(conflictindex(:,1)==dronenum);
            curconflicts=conflictindex(row,2);
            %Now, curconflicts has the index of all drones infringing on the
            %current drone. If none of the conflicts in the index involve
            %it, then this array will be empty and we'll ignore it.
            if (~isempty(curconflicts))
            %Use an algorithm like Chain, A*, or LMA here
            intermediatearray(dronenum,1:4)=PotentialFieldsZActive(dronenum,totaldronearray,curconflicts,patharray);
            end
       end
       %must create a method for drones who are avoiding
       %objectives. Right now, they pass through potential fields but not
       %GetObjective, which is where they can actually accomplish
       %objectives.
       
       %If the drone is not avoiding anything, it looks for the next point
       %along its path.
       [intermediatearray(dronenum,:),patharray{dronenum,1}]=getObjective(totaldronearray(dronenum,:),intermediatearray(dronenum,:),patharray{dronenum,1},baselength,layerMat);
       
       %move towards its objective
       totaldronearray(dronenum,:)=updatePosition(totaldronearray(dronenum,:),intermediatearray(dronenum,1:4),patharray{dronenum,1});

% %        Assorted debugging
%        if(any(totaldronearray(:,3)<-.015))
%         disp 'Underground Drone Alert'
%        end
%         
%         if (any(isnan(totaldronearray(:))))
%            disp 'NaN Alert'
%         end
% 
%        if currentIteration==stopper
% 
%            stopper=stopper+250;
%        end
% 
%         if currentIteration==20
%             [totaldronearray(1:4,1:3),objectivearray(1:4,1:3)]=debugScenario2();
%         end

    end

   %decrement stamina of drones
   totaldronearray(:,13)=totaldronearray(:,13)-1;

   %corrupt distances to find minimum non-collision distance
   distances(distances<=metalcollisiondistance)=NaN;
   mindist(currentIteration,1)=min(min(distances));
   mindist(currentIteration,2)=mean(mean(distances,'omitnan'),'omitnan');
   mindist(currentIteration,3)=median(median(distances,'omitnan'),'omitnan');
    
   %Find how many drones are in the air.
   activedrones(currentIteration)=nnz(~totaldronearray(:,12));
   
   %% If necessary, end the warmup period.

        if currentIteration==9000 %should be 9000 for a 15-minute warm up
           metalcrashes=[];
            faacrashes=0;
            conflictsrecord=0;
            objectivesreached=0;
        end
       
end
%% Begin saving
%This section saves all variable to a .m file. The
%title of the file is data, followed by parameters specific to
%each simulation. These work like this:
newname=0;
k=0;
name = 'fail'; 
while newname==0
    k=k+1;
    name=(strcat('zzz',num2str(numdrones(1)),'UAVs',num2str(input2),'dist',num2str(k),'rep.mat'));
    if ~exist((strcat('zzz',num2str(numdrones(1)),'UAVs',num2str(input2),'dist',num2str(k),'rep.mat')))
        break;
    end
end
save(name)
%now, for ease of crunching, drop the crashes into here.
numcrashes=[size(metalcrashes,1),faacrashes,conflictsrecord,input1,input2,objectivesreached]
dlmwrite('totalcrashesdata.csv',numcrashes,'-append')
exit



%    plot(linecapturex,linecapturey);
%    plot(linecapturex2,linecapturey2);
%    plot(linecapturex3,linecapturey3);


