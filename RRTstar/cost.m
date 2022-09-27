%Function that calculates the cost from vertex_X to the root of the tree S
%Uses the backpointers to parents (Does not work with trees)

function cost_to_start = cost(vertex_X,S)

%By convention, cost o v0 is 0
if vertex_X==1;
    cost_to_start=0;
    return;
end


CC=S;
k=CC.num_elements;
%Cost from parents
while k~=(1)
      CC.container(k).cost_from_parent=pdist2(cell2mat({CC.container(k).state}'),cell2mat({CC.container(CC.container(k).parent_idx).state}'));
      k=CC.container(k).parent_idx;
      

k=S.num_elements;

%Cost from Start
while k~=(1)
      CC.container(k).cost_from_start=CC.container(CC.container(k).parent_idx).cost_from_parent + CC.container(k).cost_from_parent;
      k=CC.container(k).parent_idx;

end

%Cost of the vertex to start
cost_to_start=CC.container(vertex_X).cost_from_start;


end