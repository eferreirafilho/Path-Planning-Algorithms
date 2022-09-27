%RRT with CHOMP
%Edson Bernardes - 08/11/2016

clear all
close all

%RRT 
%Size of the Configuration Space (Square)
size_x=100;
size_y=size_x;

%Define Obstacles
obstacles=[40 40 6 17;15 20 14 4; 40 15 12 6;70 70 25 10;60 80 10 20;70 30 30 20];

%Construct the Roadmap
%Create discrete map with obstacles
[map_grid]=map_with_obstacles(size_x,size_y,obstacles);

%Number of samples
num_samples=300;

%Steer fixed step size
step_size=20;

%Probability that Xrand = q_goal (Directing tree)
%Close to one -> Faster
%Close to 0 -> More chance to find path
prob_Xrand=0.8; %[0 1]


%Set q_start and q_goal
disp('Click in the figure to select q_start')
[q_start(1), q_start(2)] = ginput(1);
%Or use this:
%q_start=[46 88];
figure(1)
hold on
scatter(round(q_start(1)),round(q_start(2)),400,[0 0 0])

disp('Click in the figure to select q_goal')
[q_goal(1), q_goal(2)] = ginput(1);
%Or use this:
%q_goal=[43 33];
scatter(round(q_goal(1)),round(q_goal(2)),300,[1 0 0],'filled')

%Function that create the discrete map and run the RRT algorithm
%Returns the waypoints found by the RRT algorithm
RRT_waypoints=RRT_function(map_grid,size_x,size_y,num_samples,step_size,prob_Xrand,q_start,q_goal);

%Create Brusfire map
[brushfire_map]=create_brushfire(map_grid);

%Inverted brushfire
for i=1:size(map_grid,1)
    for j=1:size(map_grid,2)
        if map_grid(i,j)==1
            inv_map(i,j)=0;
        else
            inv_map(i,j)=1;
        end       
    end
end
[inv_brushfire_map]=create_brushfire(inv_map);
for i=1:size(inv_brushfire_map,1)
    for j=1:size(inv_brushfire_map,2)
        if inv_brushfire_map(i,j)==1
            inv_brushfire_map(i,j)=0;
        else
            inv_brushfire_map(i,j)=-inv_brushfire_map(i,j);
        end       
    end
end

%brushire + inverted brushfire
brushfire_complete=brushfire_map + inv_brushfire_map;


%Now, use CHOMP to make the  smooth if path has been found
if RRT_waypoints~= 0
    smooth_traj=chomp(RRT_waypoints,brushfire_complete);
	plot(smooth_traj(1,:), smooth_traj(2,:),'LineWidth',5)
end

 %Check smooth traj for collisions
if RRT_waypoints~= 0
    for i=1:(length(smooth_traj)-1)
        free_smooth(i)=collisionfree(smooth_traj(:,i),smooth_traj(:,i+1),map_grid);  
    end
    if sum(free_smooth)>0
           disp('Smooth Path not free')
           return
    else
           disp('Smooth Path free')
end
end




































