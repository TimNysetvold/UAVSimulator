function [] = graphDrones(totaldronearray,patharray,newcrashes,linecapture,baselength)


% each drone has is a 18-1 vector of the pattern
% [1. xposition, 2. yposition, 3. zposition,
% 4. xvel, 5. yvel, 6. zvel, 7. maxvel, 8. maxaccel
% 9. xbase, 10. ybase, 11. zbase, 12. current cruise height,
% 13. stamina, 14. type, 15. priority
% 16. separation standard, 17. pause, 18. loiter]

   hold off
   scatter3(totaldronearray(:,1),totaldronearray(:,2),totaldronearray(:,3),'ok')
%   view([0 0]);
   hold on
%    scatter3(totaldronearray(input1*2:end,1),totaldronearray(input1*2:end,2),totaldronearray(input1*2:end,3),'or')
%    scatter3(totaldronearray(4,1),totaldronearray(4,2),totaldronearray(4,3),'or')

   
   if ~isempty(newcrashes)
      scatter3(newcrashes(:,3),newcrashes(:,4),newcrashes(:,5),'xr')
   end
   
   for i=1:size(linecapture.x,1)
    plot3(linecapture.x(i,:),linecapture.y(i,:),linecapture.z(i,:));
   end
    
%    for q=1:size(patharray,1)
%         temp=patharray{q,1};
%         curobjs(q,:)=temp(1,end-3:end-1);
%    end
%    scatter3(curobjs(:,1),curobjs(:,2),curobjs(:,3),'pg')
%    viscircles([totaldronearray(:,1),totaldronearray(:,2)],ones(size(totaldronearray(:,1)))*totaldronearray(1,16));
   axis([0 baselength 0 baselength 0 1])
%      axis([4.7,5.3,4.7,5.3,.3,.7])
% axis equal
    pause(.001);
end