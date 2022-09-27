%PRM - Pre-Processing and Query
%Edson Bernardes - 24/10/2016
%Based on slide 9 of 04-PRM_RRT.pdf


function PRM_query_phase(q_start,q_goal,S,V,E,map_grid);

q_start=round(q_start);
q_goal=round(q_goal);

%Query phase - Line 1
%Add qinit to V
% Create the start vertex
v_qstart = vertex(S.get_next_idx(), q_start, 0, 0, 0, [], [], 0);

% Add the start node the container
S.add_element(v_qstart);

% Add the start node to the vertex set
V.add_vertex(v_qstart.idx);
idx_qstart=v_qstart.idx;

%Add qgoal to V
% Create the qgoal vertex
v_qgoal = vertex(S.get_next_idx(), q_goal, 0, 0, 0, [], [], 0);

% Add the goal node the container
S.add_element(v_qgoal);

% Add the goal node to the vertex set
V.add_vertex(v_qgoal.idx);
idx_qgoal=v_qgoal.idx;


%Query phase - Line 2
% Get the list of all nodes in the graph
V_list = S.get_element(V.get_idx());
   
% Compute de distance from qstart to all nodes of the graph
dist_start = pdist2(cell2mat({V_list.state}'), v_qstart.state);

%Query phase - Line 3 -qstart
[sort_dist_start,sort_index_start] = sort(dist_start,'ascend');
sort_dist_start(1)=[];
sort_index_start(1)=[];

%Query phase - Line 4 -qstart
while isempty(S.container(idx_qstart).children_set)
    i=1;
    path_is_free=collisionfree((S.container(sort_index_start(i)).state)',(q_start)',map_grid);
    if path_is_free==0
        S.container(idx_qstart).children_set=sort_index_start(i);
    end
    sort_index_start(1)=[];
    
        if isempty(sort_index_start)
    
        disp('Solution not found')
        return
    end
end


% Compute de distance from qgoal to all nodes of the graph
dist_goal = pdist2(cell2mat({V_list.state}'), v_qgoal.state);

%Query phase - Line 6 - qgoal
[sort_dist_goal,sort_index_goal] = sort(dist_goal,'ascend');
sort_dist_goal(1)=[];
sort_index_goal(1)=[];

%Query phase - Line 4 -qgoal
while isempty(S.container(idx_qgoal).children_set)
    i=1;
    path_is_free=collisionfree((S.container(sort_index_goal(i)).state)',(q_goal)',map_grid);
    if path_is_free==0
        S.container(idx_qgoal).children_set=sort_index_goal(i);
        S.container(sort_index_goal(i)).children_set=idx_qgoal;
       
    end
    sort_index_goal(1)=[];
    if isempty(sort_index_goal)
    
        disp('Solution not found')
        return
    end
    
    
end






          
CC=search(idx_qstart,S); % Calcula a �ravore BFS from qstart

% for i=1:length(S.container)
%     CC.container(i).cost_from_parent=pdist2(cell2mat({CC.container(i).state}'),cell2mat({CC.container(CC.container(1).parent_idx).state}'));
%     CC.container(i).cost_from_start=pdist2(cell2mat({CC.container(i).state}'),cell2mat({CC.container(idx_qstart).state}'));
% 
% 
% end



for i=1:CC.num_elements
    vi=CC.get_element(i);
    vj=S.get_element(CC.container(i).parent_idx);
    figure(1)
    hold on
    plot([vi.state(1) vj.state(1)], [vi.state(2) vj.state(2)], 'Color','black','LineWidth',3)
   
end

%Shortest Path
%Query phase - Line 7 -qstart

%Shortest path, number of nodes
k=CC.num_elements;
i=1;
while k~=(CC.num_elements-1)
  path_BFS(:,i)=CC.container(CC.container(k).parent_idx).state;
  k=CC.container(k).parent_idx;
  i=i+1;
end

%Add q_goal to path
path_BFS=[q_goal' path_BFS];
%Plot path
plot(path_BFS(1,:),path_BFS(2,:),'LineWidth',5) 
        


end



