%RRG
%Edson Bernardes - 15/11/2016
%Based on slide 14 04-PRMStar_RRTStar.pdf

%Construct RRG graph
%Added Probability to 'draw' q_goal
%Added stop criteria (Not in original algorithm)
%Search best path in the constructed graph


clear all
close all

%Size of the Configuration Space (Square)
size_x=200;
size_y=size_x;

%Define Obstacles
obstacles=[40 40 6 17;15 20 14 4; 40 15 12 6;70 70 25 10;60 80 10 20;70 30 30 20;150 150 30 30;150 10 40 20;30 160 40 60];

%Number of samples
num_samples=200;

%Steer step size (Variable inside loop)
step_size=50;

%Probability that Xrand = q_goal (Directing tree)
%Close to one -> Faster
%Close to 0 -> More chance to find path
prob_Xrand=0.3; %[0 1]

%Construct the Roadmap
%Create discrete map with obstacles
[map_grid]=map_with_obstacles(size_x,size_y,obstacles);

%Increase Factor
inc_factor=size_x;

%Set q_start and q_goal
disp('Click in the figure to select q_start')
[q_start(1), q_start(2)] = ginput(1);
%Or use this:
%q_start=[2 2];
figure(1)
hold on
scatter(round(q_start(1)),round(q_start(2)),400,[0 0 0])

disp('Click in the figure to select q_goal')
[q_goal(1), q_goal(2)] = ginput(1);
%Or use this:
%q_goal=[100 65];
scatter(round(q_goal(1)),round(q_goal(2)),300,[1 0 0],'filled')

% Create the container for all nodes
S = container_set(vertex.empty());
% Create the node set - This contains only indexes for the nodes. The
% actual nodes are stored in the container set
V = vertex_set();
% Create de edge set
E = edge_set();

rng(100); % Seed of random number generator



%Algorithm RRT - line 1
% Create the start node with the first start point
v_start = vertex(S.get_next_idx(), round(q_start), 0, 0, 0, [], [], 0);
% Add the start node the container
S.add_element(v_start);
% Add the start node to the vertex set
V.add_vertex(v_start.idx);


path_found=0;

gamma_gain=0.1;

%Variable Radius Calculation
dim=2;%Dimension
vol_x_free=size_x*size_y;%Aproximation
vol_unit_ball=(pi*1)^2;%Only for dimension=2

gamma_prm_star=2*((1+(1/dim))^(1/2))*((vol_x_free/vol_unit_ball)^(1/2));
gamma_prm=gamma_prm_star + gamma_gain;
        



%Algorithm RRT - line 2
for i=1:num_samples
    if (path_found==0)
        
    %Algorithm RRT - line 3
    %Generate random node
    Xrand_aux=1;
    while (Xrand_aux==1)
        if rand>prob_Xrand
            Xrand=(inc_factor-1)*[rand, rand] +1;
        else
            Xrand=q_goal;%Directing tree to q_goal
        end
            Xrand=round(Xrand);
            Xrand_aux=sample_free(Xrand(1),Xrand(2),map_grid);
    end
    
    %Algorithm RRT - line 4
    Xnearest_idx=nearest(S,V,Xrand);
    Xnearest=S.container(Xnearest_idx).state;
    
    %Algorithm RRT - line 5
    Xnew=steer(Xnearest,Xrand,step_size);
    
    %Make sure Xnew is inside map
        if Xnew(1)<=1
            Xnew(1)=1;
        end
        if Xnew(1)>=size_x
            Xnew(1)=size_x-1;
        end
        if Xnew(2)<=1
            Xnew(2)=1;
        end
        if Xnew(2)>=size_y
            Xnew(2)=size_y-1;
        end
   
        
   % Create the start node with the first start point
   v_new = vertex(S.get_next_idx(), Xnew, 0, 0, 0, [], [], 0);
        
    
    %Algorithm RRG - line 6
    obstaclesfree=collisionfree(Xnearest',Xnew',map_grid);
    if obstaclesfree==0 && path_found==0
        
        
        % Get the list of all nodes in the graph
        V_list = S.get_element(V.get_idx());
        % Compute de distance from v_new to all nodes of the graph
        dist = pdist2(cell2mat({V_list.state}'), v_new.state);

        %Algorithm RRG - line 7
        cardV=size(V_list,2);
       
        min_gamma=min([gamma_prm*((log(cardV)/cardV)^(1/dim)) step_size]);
        Xnear=near(dist,min_gamma,V_list);
        
        
        %Algorithm RRG - line 8
        % Add the new node the container
        S.add_element(v_new);
        % Add the new node to the vertex set
        V.add_vertex(v_new.idx);
        % Create edges for from each element in V_near to v_new and
        % vice-versa
        E.add_edge(S.container(Xnearest_idx).idx , v_new.idx);
        E.add_edge(v_new.idx , S.container(Xnearest_idx).idx);
        
        
        
        % Set parent for the new element        
        S.container(v_new.idx).children_set=[S.container(Xnearest_idx).children_set, v_new.idx];
        S.container(Xnearest_idx).children_set=[S.container(v_new.idx).children_set, Xnearest_idx];
                    
   
        i=1;
        %Algorithm RRG - line 9
        while i~=size(Xnear,2)+1 && path_found==0
                x_near=Xnear(i).state;
                obstaclesfree=collisionfree(x_near',Xnew',map_grid);
                if obstaclesfree==0
                	x_near_idx=Xnear(i).idx;
                    E.add_edge(x_near_idx , v_new.idx);
                    %Set X_near as children of v_new
                    S.container(v_new.idx).children_set=[S.container(x_near_idx).children_set, v_new.idx];

                                    
                        %Stop Criteria (Not in the original algorithm)
                        %If its close to q_goal, connect q_goal to graph and stop
                        if pdist2(v_new.state,round(q_goal),'euclidean')<=step_size/3
                            obstaclesfree=collisionfree(v_new.state',round(q_goal)',map_grid);
                            if obstaclesfree==0 
                            % Create node for q_goal
                            v_goal = vertex(S.get_next_idx(), round(q_goal), 0, 0, 0, [], [], 0);
                            % Add q_goal node the container
                            S.add_element(v_goal);
                            % Add q_goal node to the vertex set
                            V.add_vertex(v_goal.idx);
                            % Create edges for from each element in V_near to v_new
                            E.add_edge(v_goal.idx, v_new.idx);

                            % Set children for q_goal element            
                            S.container(v_goal.idx).children_set=[S.container(v_new.idx).children_set, v_goal.idx];
                          

                            %Stop
                            path_found=1;
                            end
                 
                          end
   
                end
                i=i+1;
        end

    end
    
    end
end


%BFS search for best path (In number of nodes)
CC = search(v_start.idx, S);

% Plot Graph
figure(1)
hold on
axis([0 inc_factor*1 0 inc_factor*1])
for i=1:S.num_elements()
    for j=1:S.num_elements()
        if E.has_edge(i,j)
            vi=S.get_element(i);
            vj=S.get_element(j);
            plot([vi.state(1) vj.state(1)], [vi.state(2) vj.state(2)], 'o', 'Color','blue')
            plot([vi.state(1) vj.state(1)], [vi.state(2) vj.state(2)], 'Color','black')
            axis([0 inc_factor*1 0 inc_factor*1])
        end
    end
   
end

%If path not found
if S.container(length(S.container)).state~=round(q_goal)
     disp('Path not Found!')
else %If path found
        disp('Path Found!')
           %Shortest path, number of nodes
        k=CC.num_elements;
        i=1;
        while k~=1%(CC.num_elements-1)
        %for k=1:5   

          path_BFS(:,i)=CC.container(CC.container(k).parent_idx).state;
          k=CC.container(k).parent_idx;
          i=i+1;

          
          
        end
        
       %Add q_goal to path
       path_BFS=[ q_goal' path_BFS];
       %Plot path
       plot(path_BFS(1,:),path_BFS(2,:),'LineWidth',3,'Color','Green') 
end


  



















    





















