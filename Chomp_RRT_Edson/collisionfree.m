%Function to verify if a path is collision free
%Edson Bernardes Ferreira Filho
%23/10/2016

%Incremental Algorithm (04-PRM_RRT-Guilherme Pereira - Slide 12)
%Returns failure if path is blocked.


function [path_is_free] = collisionfree(point1,point2,map_grid)
 %point1=[1;1]
 
 point1=round(point1);
 point2=round(point2);
 
%Check points consistency
 if ~isequal(size(point1),[2 1]) || ~isequal(size(point2),[2 1])
     disp('Points must be a 2x1 vector')
    path_is_free='NaN';
     return
 end
 if point1(1)<=0 || point1(1)>=size(map_grid,1) || point2(1)<=0 || point2(1)>=size(map_grid,2) || point1(2)<=0 || point1(2)>=size(map_grid,1) || point2(2)<=0 || point2(2)>=size(map_grid,2)
     disp('Points are not defined be on the border and outside the map')
    path_is_free='NaN';
     return
 end

 
%Create a map_grid that the line connecting two points occupy
size_x=size(map_grid,1);
size_y=size(map_grid,2);
 

map_grid(size_x,size_y)=0;

    %Direction of the line connecting 2 points
    direction=point2-point1;
    %Normalize the direction
    direction_norm=direction/norm(direction,inf);



 
%while(int_point(point_num,:)~=point2)
   
%new_check_rounded=point1;
new_check=point1;
while(norm(point1-point2)~=0)
    
    %Direction of the line connecting 2 points
    direction=point2-new_check;
    %Normalize the direction
    direction_norm=direction/norm(direction,inf);
    
    %New point to check
    new_check=point1+direction_norm;
    %Round the point (discretization)
    %The randn is added so that sometimes it round down and sometimes it
    %round up
    new_check_rounded=round(new_check);
    
    %We must round the same way when moving the robot
    %new_check_rounded=round(new_check);
    point1=new_check;

    %If the path is not free
    if map_grid(new_check_rounded(1),new_check_rounded(2))==1
        path_is_free=1;
        return
    end
    
    
    %Draw the path until it collides, if it collides
    %Disabled by default
    %figure(1)
    %hold on
    %scatter(new_check_rounded(1),new_check_rounded(2),'b')

end
%If the path is free
path_is_free=0;

end