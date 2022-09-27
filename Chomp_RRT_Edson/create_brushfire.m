%Function that creates a brushfire map
%Edson Bernardes - 05/11/2016


function [brushfire_map]=create_brushfire(map_grid)

brushfire_map=map_grid;

for i=1:size(brushfire_map,1)
    for j=1:size(brushfire_map,2)
        %Setting '1' to borders of the map
        if i<=1 || j<=1 || i>=size(map_grid,1) || j>=size(map_grid,2)
            brushfire_map(i,j)=1;
        end
    end
end
   
%Swipe from top to bottom
n=2;
for i=1:size(brushfire_map,1)
    for j=1:size(brushfire_map,2)
        %Calculate neighboors of point i,j
        %Connectivity 4
            neighboors=[i+1 j;i j+1;i-1 j;i j-1];
            for k=1:4 
                %Inside map
                if neighboors(k,1)<size(map_grid,1) && neighboors(k,2)<size(map_grid,2) && neighboors(k,1)>0 && neighboors(k,2)>0
                    %Equal to zero or smaller value
                    if brushfire_map(neighboors(k,1),neighboors(k,2))==0 || (brushfire_map(i,j)+1)<brushfire_map(neighboors(k,1),neighboors(k,2))
                        brushfire_map(neighboors(k,1),neighboors(k,2))=brushfire_map(i,j)+1;
                    end
              
                end

            end
   
       
    end
end


%Swipe again, now from bottom to top
n=2;
for i=100:-1:1
    for j=100:-1:1
        %Calculate neighboors of point i,j
            neighboors=[i+1 j;i j+1;i-1 j;i j-1];
            for k=1:4 
                %Inside map
                if neighboors(k,1)<size(map_grid,1) && neighboors(k,2)<size(map_grid,2) && neighboors(k,1)>0 && neighboors(k,2)>0
                    %Equal to zero or smaller value
                    if brushfire_map(neighboors(k,1),neighboors(k,2))==0 || (brushfire_map(i,j)+1)<brushfire_map(neighboors(k,1),neighboors(k,2))
                        brushfire_map(neighboors(k,1),neighboors(k,2))=brushfire_map(i,j)+1;
                    end
              
                end

            end
         
    end
end









