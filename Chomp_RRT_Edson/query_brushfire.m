
function [dist, gradi_brush]=query_brushfire(brushfire_map, point)


%Treating border points
if point(1)<=1
    point(1)=2;
end
if point(1)>=size(brushfire_map,1)
   point(1)=size(brushfire_map,1)-1;
end
if point(2)<=1
    point(2)=2;
end
if point(2)>=size(brushfire_map,2)
   point(2)=size(brushfire_map,2)-1;
end


%Distance from closer obstacle
dist=brushfire_map(point(1),point(2));


%Calculate neighboors of point
%Connectivity 4

%4 neighboors
    neighboors(1)=brushfire_map((point(2)),(point(1)+1));
    neighboors(2)=brushfire_map((point(2)+1),(point(1)));
    neighboors(3)=brushfire_map((point(2)),(point(1)-1));
    neighboors(4)=brushfire_map((point(2))-1,(point(1)));
   
    
    
    %Searching neighboor with bigger value
    [Y I] = sort(neighboors); 


%Gradient
%Direction to furthest obstacle
if I(4)==1
    point_aux=[point(2) , point(1)-1];
end
if I(4)==2 
    point_aux=[point(2)+1 , point(1)];
end
if I(4)==3
    point_aux=[point(2) , point(1)+1];
end
if I(4)==4 
    point_aux=[point(2)-1 , point(1)];
end
 
%Gradient
gradi_brush=flip(point_aux')-point';



end