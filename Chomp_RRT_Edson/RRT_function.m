%RRT - Function
%Edson Bernardes - 03/11/2016
%Based on slides 15 to 17 of 04-PRM_RRT.pdf



function [RRT_waypoints] = RRT_function(map_grid, size_x,size_y,num_samples,step_size,prob_Xrand,q_start,q_goal)



%Increase Factor
inc_factor=size_x;

%Map is discrete
q_start=round(q_start)
q_goal=round(q_goal)

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
v_start = vertex(S.get_next_idx(), q_start, 0, 0, 0, [], [], 0);
% Add the start node the container
S.add_element(v_start);
% Add the start node to the vertex set
V.add_vertex(v_start.idx);


path_found=0;

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
        
    
    %Algorithm RRT - line 6
    obstaclesfree=collisionfree(Xnearest',Xnew',map_grid);
    if obstaclesfree==0
        
        if pdist2(v_new.state,round(q_goal),'euclidean')<=step_size/2;
            path_found=1;
            v_new = vertex(S.get_next_idx(), q_goal, 0, 0, 0, [], [], 0);           
        end
        
        %Algorithm RRT - line 7
        % Add the new node the container
        S.add_element(v_new);
        % Add the new node to the vertex set
        V.add_vertex(v_new.idx);
        % Create edges for from each element in V_near to v_new
        E.add_edge(S.container(Xnearest_idx).idx, v_new.idx);
        % Set parent for the new element        
        S.container(v_new.idx).parent_idx=Xnearest_idx;
        

        
        
    end
    
    
    
    
    
    
    
    
    
    
    %Stop Criteria (Not in the original algorithm)
    %If its close to q_goal, connect q_goal to graph and stop
%     if pdist2(v_new.state,round(q_goal),'euclidean')<=step_size/2
%             obstaclesfree=collisionfree(v_new.state',q_goal',map_grid);
%     if obstaclesfree==0
%         % Create node for q_goal
%         v_goal = vertex(S.get_next_idx(), round(q_goal), 0, 0, 0, [], [], 0);
%         % Add q_goal node the container
%         S.add_element(v_goal);
%         % Add q_goal node to the vertex set
%         V.add_vertex(v_goal.idx);
%         % Create edges for from each element in V_near to v_new
%         E.add_edge(v_goal.idx, v_new.idx);
%         % Set parent for q_goal element           
%         S.container(v_goal.idx).parent_idx=Xnearest_idx;
%          
%         %Stop
%         path_found=1;
%     end
%     end
    end
end

% if v_new.idx==v_goal.idx  
%     disp('fdfs')
%     %v_new=v_goal;
% end

%If path not found
%if S.container(length(S.container)).state~=round(q_goal)
if path_found==0    
     disp('Path not Found!')
     path_to_goal=0;
else %If path found
        disp('Path Found!')
        k=S.num_elements;
        i=1;
        %Path is just the backpointers from q_goal to q_start
        while k~=(1)
            
             
            path_to_goal(:,i)=S.container(S.container(k).parent_idx).state;
            k=S.container(k).parent_idx;
            i=i+1; 
              
              
          %end
        end

        %Add q_goal to path
        path_to_goal=[q_goal' path_to_goal];
        %Plot path
        %plot(path_to_goal(1,:),path_to_goal(2,:),'LineWidth',5) 

end


  
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

RRT_waypoints=path_to_goal;



















    





















