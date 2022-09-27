%Function Sample Free
%Edson Bernardes Ferreira Filho
%22/10/2016

function [point_is_free]=sample_free(x,y,map_grid)

%Check if point is inside map grid
if x>size(map_grid,1) || y>size(map_grid,2)
    disp('Point outside map grid')
return
end

%Check if point is a integer
if isinteger(x) || isinteger(y)
    disp('Point must be a integer')
return
end


%Position X,Y is free
if map_grid(x,y)==0;
     point_is_free=0;
end

%Position X,Y is NOT free
if map_grid(x,y)==1;
     point_is_free=1;
end

 
end
