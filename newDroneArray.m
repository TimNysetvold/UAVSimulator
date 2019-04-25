function dronearray = newDroneArray(numdrones, maxvel, maxaccel, infringe, priority, type,stamina,xbase,ybase,pause,baselength,loiter,cruiseheight)

%Creates a new drone array from input parameters.
% each drone has is an 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

addvector=ones(numdrones,1);
zerovector=zeros(numdrones,1);
cruiseheight=addvector*cruiseheight;

xposvec=baselength*rand(numdrones,1);
yposvec=baselength*rand(numdrones,1);
zposvec=cruiseheight;

xvelvec=zerovector;
yvelvec=zerovector;
zvelvec=zerovector;
maxvelvec=addvector*maxvel;

maxaccelvec=maxaccel*addvector;

infringevec=infringe*addvector;
staminavec=stamina*ones(numdrones,1)*3000;
priority=addvector*priority;
type=addvector*type;

%For now, bases will be randomly assigned.
xbasevec=baselength*rand(numdrones,1);
ybasevec=baselength*rand(numdrones,1);
% xbasevec=baselength*.5*ones(numdrones,1);
% ybasevec=baselength*.5*ones(numdrones,1);
zbasevec=addvector*0;

loitervec=addvector*0;
if type==3
    xbasevec=xposvec;
    ybasevec=yposvec;
    
end
pausevec=addvector*pause;

% each drone has is a 21-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

dronearray=[xposvec,yposvec,zposvec,xvelvec,yvelvec,zvelvec,maxvelvec,maxaccelvec,xbasevec,ybasevec,zbasevec,cruiseheight,staminavec,type,priority,infringevec,pausevec,loitervec];





end
