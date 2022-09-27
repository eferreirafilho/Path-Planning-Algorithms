%PRM - Pre-Processing and Query
%Edson Bernardes - 24/10/2016
%Based on slides 6 and 9 of 04-PRM_RRT.pdf

clear all
close all

%Size of the Configuration Space (Square)
size_x=50;
size_y=size_x;

%Define Obstacles
obstacles=[40 40 6 17;15 20 14 4; 40 15 12 6];

%Number of samples
num_samples=20;

%Construct the Roadmap
[S,V,E, map_grid]=PRM_pre_phase(size_x,size_y,obstacles,num_samples);

%QUERY PHASE
%Search for path

disp('Click in the figure to select q_start')
[q_start(1), q_start(2)] = ginput(1);
%Or use this:
%q_start=[1 1];
figure(1)
hold on
scatter(q_start(1),q_start(2),400,[0 0 0])

disp('Click in the figure to select q_goal')
[q_goal(1), q_goal(2)] = ginput(1);
%Or use this:
%q_goal=[49 49];
scatter(q_goal(1),q_goal(2),300,[1 0 0],'filled')

PRM_query_phase(q_start,q_goal,S,V,E,map_grid);


