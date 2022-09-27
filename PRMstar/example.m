%PRM* - Pre-Processing and Query
%Edson Bernardes - 11/11/2016
%Based on slides 6 and 9 of 04-PRM_RRT.pdf 
%and slides 12 to 14 of 08-PRMStar_RRTStar.pdf


%Construct PRM* graph
%Added Probability to 'draw' q_goal
%Added stop criteria (Not in original algorithm)
%Search best path in the constructed graph


clear all
close all

%Size of the Configuration Space (Square)
size_x=100;
size_y=size_x;

%Define Obstacles
obstacles=[40 40 6 17;15 20 14 4; 40 15 12 6;60 50 25 36];

%Number of samples
num_samples=30;

%Parameter added to gamma_star to guarantee asymptotic optimality of PRM*
%Greater -> More dense Graph (More connections)
gamma_gain=20;


%Construct the Roadmap
[S,V,E, map_grid]=PRMstar_pre_phase(size_x,size_y,obstacles,num_samples,gamma_gain);

%QUERY PHASE
%Search for path

disp('Click in the figure to select q_start')
[q_start(1), q_start(2)] = ginput(1);
%Or use this:
%q_start=[1 1];
figure(1)
hold on
scatter(q_start(1),q_start(2),200,[0 0 0])

disp('Click in the figure to select q_goal')
[q_goal(1), q_goal(2)] = ginput(1);
%Or use this:
%q_goal=[49 49];
scatter(q_goal(1),q_goal(2),100,[1 0 0],'filled')

PRM_query_phase(q_start,q_goal,S,V,E,map_grid);


