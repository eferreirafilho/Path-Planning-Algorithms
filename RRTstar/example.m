%RRT*
%Edson Bernardes - 16/11/2016
%Based on slide 18 to 25 of 08-PRMStar_RRTStar.pdf

%Construct RRT* tree
%Added Probability to 'draw' q_goal
%Added stop criteria (Not in original algorithm)
%Get best path from the constructed tree

clear all
close all

%Size of the Configuration Space (Square)
size_x=500;
size_y=size_x;

%Define Obstacles
obstacles=5*[40 40 6 17;15 20 14 4; 40 15 12 6;70 70 25 10;60 80 10 20;70 30 30 20];

%Number of samples
num_samples=450;

%Steer fixes step size
step_size=30;

%Probability that Xrand = q_goal (Directing tree)
%Close to one -> Faster
%Close to 0 -> More chance to find path
prob_Xrand=0; %[0 1]

%Construct the Roadmap
%Create discrete map with obstacles
[map_grid]=map_with_obstacles(size_x,size_y,obstacles);

%Increase Factor
inc_factor=size_x;

%Set q_start and q_goal
disp('Click in the figure to select q_start')
[q_start(1), q_start(2)] = ginput(1);
%Or use this:
%q_start=[1 1];
figure(1)
hold on
scatter(round(q_start(1)),round(q_start(2)),400,[0 0 0])

disp('Click in the figure to select q_goal')
[q_goal(1), q_goal(2)] = ginput(1);
%Or use this:
%q_goal=[98 64];
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


gamma_gain=0.1;

%Variable Radius Calculation
dim=2;%Dimension
vol_x_free=size_x*size_y;%Aproximation
vol_unit_ball=(pi*1)^2;%Only for dimension=2

gamma_prm_star=2*((1+(1/dim))^(1/2))*((vol_x_free/vol_unit_ball)^(1/2));
gamma_prm=gamma_prm_star + gamma_gain;
        


path_found=0;

%Algorithm RRT* - line 2
for i=1:num_samples
    if (path_found==0)
    %Algorithm RRT* - line 3
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
    
    
    %Algorithm RRT* - line 4
    Xnearest_idx=nearest(S,V,Xrand);
    Xnearest=S.container(Xnearest_idx).state;
    
    %Algorithm RRT* - line 5
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
   
        
    
    
    %Algorithm RRT* - line 6
    obstaclesfree=collisionfree(Xnearest',Xnew',map_grid);
    if obstaclesfree==0 
        
        %Algorithm RRT* - line 7
        % Create node with Xnew
        v_new = vertex(S.get_next_idx(), Xnew, 0, 0, 0, [], [], 0);
        
        % Get the list of all nodes in the graph
        V_list = S.get_element(V.get_idx());
        % Compute the distance from v_new to all nodes of the graph
        dist = pdist2(cell2mat({V_list.state}'), v_new.state);

        cardV=S.num_elements;
        min_gamma=min([gamma_prm*((log(cardV)/cardV)^(1/dim)) step_size]);
        %Set of near points:
        Xnear=near(dist,min_gamma,V_list);
        
        %Algorithm RRT* - line 8

        % Add the new node the container
        S.add_element(v_new);
        % Add the new node to the vertex set
        V.add_vertex(v_new.idx);

        % Set parent for the new element        
        S.container(v_new.idx).parent_idx=Xnearest_idx;
        
        
         %Algorithm RRT* - line 9
         Xmin_idx=Xnearest_idx;
         cmin=cost(Xnearest_idx,S) + cLine(Xnearest,Xnew);
         
         j=1;
         %Algorithm RRT* - line 10
         while j~=size(Xnear,2)+1
             x_near=Xnear(j).state;
             obstaclesfree=collisionfree(x_near',Xnew',map_grid);
              %Algorithm RRT* - line 11        
             if obstaclesfree==0 && (cost(Xnear(j).idx,S)+cLine(x_near,Xnew))<cmin && ~isequal(Xnew,x_near)
                 %Algorithm RRT* - line 12       
                 Xmin_idx=Xnearest_idx;
                 cmin=cost(Xnear(j).idx,S) + cLine(x_near,Xnew);      
             end
             j=j+1;
         end
         %Algorithm RRT* - line 13       
         E.add_edge(Xmin_idx, v_new.idx);

         %Algorithm RRT* - line 14
         %Rewire the tree
         j=1;
         while j~=size(Xnear,2)+1
             x_near=Xnear(j).state;
             obstaclesfree=collisionfree(x_near',Xnew',map_grid);
              %Algorithm RRT* - line 15     
             if obstaclesfree==0 && (cost(v_new.idx,S)+cLine(x_near,Xnew))<(cost(Xnear(j).idx,S)) && ~isequal(x_near,v_new.state)
                
                 
                 %Algorithm RRT* - line 16
                 x_parent=S.get_element(Xnear(j).parent_idx);
                 E.remove_edge(x_parent.idx,Xnear(j).idx);
                 
         
                 x_near_childrenset = find(x_parent.children_set == Xnear(j).idx);
                 x_parent.children_set(x_near_childrenset) = [];
                 Xnear(j).parent_idx=v_new.idx;
                 E.add_edge(v_new.idx,Xnear(j).idx);
                 S.container(v_new.idx).children_set=[S.container(v_new.idx).children_set, Xnear(j).idx];
                 S.container(Xnear(j).idx).parent_idx=v_new.idx;
                 
                 updateChildCosts(v_new.idx,S,V);
                 S.container(v_new.idx).cost_from_start=cost(v_new.idx,S);
                 
                 
                 
                 
             
             end
             j=j+1;
         end
             %Stop Criteria (Not in the original algorithm)
    %If its close to q_goal, connect q_goal to graph and stop
    if pdist2(v_new.state,round(q_goal),'euclidean')<=step_size
        % Create node for q_goal
        v_goal = vertex(S.get_next_idx(), round(q_goal), 0, 0, 0, [], [], 0);
        % Add q_goal node the container
        S.add_element(v_goal);
        % Add q_goal node to the vertex set
        V.add_vertex(v_goal.idx);
        % Create edges for from each element in V_near to v_new
        E.add_edge(v_goal.idx, v_new.idx);
        % Set parent for q_goal element            
        S.container(v_goal.idx).parent_idx=v_new.idx;
         
        %Stop
        path_found=1;
    end
         
         
         end

         
         
          
    end
    
    
   
%     
% % Plot Graph
% figure(1)
% hold on
% axis([0 inc_factor*1 0 inc_factor*1])
% for i1=1:S.num_elements()
%     for i2=1:S.num_elements()
%         if E.has_edge(i1,i2)
%             vi=S.get_element(i1);
%             vj=S.get_element(i2);
%             plot([vi.state(1) vj.state(1)], [vi.state(2) vj.state(2)], 'o', 'Color','blue')
%             plot([vi.state(1) vj.state(1)], [vi.state(2) vj.state(2)], 'Color','black')
%             axis([0 inc_factor*1 0 inc_factor*1])
%         end
%     end
%    
% end
% drawnow




    
 
end



%If path not found
if S.container(length(S.container)).state~=round(q_goal)
     disp('Path not Found!')
else %If path found
        disp('Path Found!')
        k=S.num_elements;
        i=1;
        %Path is just the backpointers from q_goal to q_start
        while k~=(1)
          path_to_goal(:,i)=S.container(S.container(k).parent_idx).state;
          k=S.container(k).parent_idx;
          i=i+1;
        end

        %Add q_goal to path
        path_to_goal=[q_goal' path_to_goal];
        %Plot path
        plot(path_to_goal(1,:),path_to_goal(2,:),'LineWidth',5) 

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









    





















