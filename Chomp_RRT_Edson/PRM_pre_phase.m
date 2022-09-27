% Example of use for graph classes
% Guilherme Pereira - 17/10/2016
% PRM algorithm - Slide 6 and Slide 9 of 04-PRM-RRT.pdf
% Edson Bernardes - 24/10/2016
% Construct the Roadmap

function [S,V,E,map_grid] = PRM_pre_phase(size_x,size_y,obstacles,n)

%Create discrete map with obstacles
[map_grid]=map_with_obstacles(size_x,size_y,obstacles);

%Increase Factor
inc_factor=size_x;

% Create the container for all nodes
S = container_set(vertex.empty());

% Create the node set - This contains only indexes for the nodes. The
% actual nodes are stored in the container set
V = vertex_set();

% Create de edge set
E = edge_set();

rng(100); % Seed of random number generator

% Generate the first random node
new_start_point_free=1;
while (new_start_point_free==1)   %Algorithm 1 - PRM - line 3
        new_start=(inc_factor-1)*[rand, rand] +1;
        new_start=round(new_start);
        new_start_point_free=sample_free(new_start(1),new_start(2),map_grid);
end

% Create the start node with the first random node
v_start = vertex(S.get_next_idx(), new_start, 0, 0, 0, [], [], 0);

% Add the start node the container
S.add_element(v_start);

% Add the start node to the vertex set
V.add_vertex(v_start.idx);

% Create a graph
for i=1:n   %Algorithm 1 - PRM - line 2 
    
   % Generate a new random node
   new_point_free=1;
   while (new_point_free==1)   %Algorithm 1 - PRM - line 3 -> xrand
        new=(inc_factor-2)*[rand, rand] +1;
        new=round(new);
        new_point_free=sample_free(new(1),new(2),map_grid);
   end

   v_new = vertex(S.get_next_idx(), new, 0, 0, 0, [], [], 0);
   
   % Connect the new node to the nodes close to it if there is one
   % Get the list of all nodes in the graph
   V_list = S.get_element(V.get_idx());
        
   %Algorithm 1 - PRM - line 4
   % Compute de distance from v_new to all nodes of the graph
   dist = pdist2(cell2mat({V_list.state}'), v_new.state);
        
   % Create the list of all nodes close to v_new
   %Algorithm 1 -> U
   V_near=near(dist,inc_factor*0.5,V_list);
    
      
    %Algorithm 1 - PRM - line 5 - V  <- V U xrand
    S.add_element(v_new);
    V.add_vertex(v_new.idx);
    
   %Algorithm 1 - PRM - line 6
   for i=1:length(V_near)
       
            %Algorithm 1 - PRM - line 8
            path_is_free=collisionfree((V_near(i).state)',(v_new.state)',map_grid);
            if path_is_free==0
            % Create edges for from each element in V_near to v_new
            E.add_edge(V_near(i).idx, v_new.idx);
            % Add v_new to the list of neighbors of each element of V_near
            S.container(V_near(i).idx).children_set=[S.container(V_near(i).idx).children_set, v_new.idx];
            % Add each element of V_near to the list of neighbors of v_new
            S.container(v_new.idx).children_set=[S.container(v_new.idx).children_set, V_near(i).idx];
            end
   end
   
end

% Plot
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






