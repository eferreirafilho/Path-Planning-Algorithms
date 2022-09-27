%Function that calculates the cost from vertex_X to the root of the tree S
%Uses the backpointers to parents (Does not work with trees)

function cost_to_start = cost(vertex_X,S)

%By convention, cost of v0 is 0
if vertex_X==1;
    cost_to_start=0;
    return;
end


CC=S;

k=vertex_X;
cost_aux=0;

%Cost from Start
while k~=(1)

      cost_aux=cost_aux+CC.container(k).cost_from_parent;
      k=CC.container(k).parent_idx;
     
    
end

cost_to_start=cost_aux;


end