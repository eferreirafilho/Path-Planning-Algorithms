%Function to create a map with obstacles
%Edson Bernardes Ferreira Filho
%22/10/2016

%CONFIGURATION SPACE
%x_max and y_max are the maximum coordinates of the configuration space
%Config. Space always start in (0,0)
%Not defined in the borders!

%OBSTACLES
%Obstacles is a (n_obst x 4) matrix
%Each line of the matrix have 4 values to represent start and end of an
%obstacle
%Obstacles are always retangular
%One obstacle has the format -> [x y width height]

%RESOLUTION
%Resolution of the grid is fixed always equal to 1
%To 'change' the resolution, increase or decrease the limits of the
%configuration space

%FUNCTION CALL EXAMPLE
%OBST=[2 5 6 7;15 2 4 4; 11 15 2 6]
%map_with_obstacles(20,20,OBST)
%Will return the map grid and will plot the obstacles


function [map_grid]=map_with_obstacles(x_max,y_max,obstacles)

%Total number of obstacles (retangular)
n_obst=size(obstacles,1);

%Check the obstacles matrix
if size(obstacles,2)~=4 
    disp('Obstacles especifications are wrong! Must have 4 Values in each line!')
    return
end

%Check if obstacles are inside config. space.
%If not, 'cut' the obstacles
for i=1:n_obst
if obstacles(i,1)<0
    obstacles(i,1)=0;
end
if obstacles(i,2)<0
    obstacles(i,2)=0;
end
%Obstacles are in format [x y width height]
if obstacles(i,1)+obstacles(i,3)>x_max
    obstacles(i,3)=x_max-obstacles(i,1);
end
if obstacles(i,2)+obstacles(i,4)>y_max
    obstacles(i,4)=y_max-obstacles(i,2);
end 
%Obstacles need to be occupy at least one space in the grid
if obstacles(i,3)<=0
    obstacles(i,3)=1;
end
if obstacles(i,4)<=0
    obstacles(i,4)=1;
end
end



%For each obstacle, plot it
for i=1:n_obst
    axis([0 x_max 0 y_max])
    rectangle('Position',obstacles(i,1:4),'FaceColor',[0.5 .1 .1])
end

%Set plot grid lines with 1 resolution
%This code to create grid lines was copied from matlab answers
hold on
g_x=[0:1:x_max]; % user defined grid Y [start:spaces:end]
g_y=[0:1:y_max]; % user defined grid X [start:spaces:end]
for i=1:length(g_x)
   plot([g_x(i) g_x(i)],[g_y(1) g_y(end)],'k:') %y grid lines
   hold on    
end
for i=1:length(g_y)
   plot([g_x(1) g_x(end)],[g_y(i) g_y(i)],'k:') %x grid lines
   hold on    
end
%This code to create grid lines was copied from matlab answers


%Create map grid (resolution fixed=1)
%Zeros are empty space and ones are spaces with obstacles
%Fill it with zeros
for i=1:x_max
for j=1:y_max
    map_grid(i,j)=0;
end
end

%Fill with ones in the obstacles
for i=1:x_max
for j=1:y_max
    for k=1:n_obst
        if  i>=obstacles(k,1) && i<=obstacles(k,1)+obstacles(k,3) && j>=obstacles(k,2) && j<=obstacles(k,2)+obstacles(k,4)
            map_grid(i,j)=1;
        end
        
    end
end
end



%DELETE!
%The map_grid is somewhat flipped from the plot
%Flip the map grid to match the plot
%map_grid=flip(map_grid);
%S.get_element(2).state(1)


end



