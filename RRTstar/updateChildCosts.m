function [S]= updateChildCosts(vnew,S,V)


m=S.container(vnew).children_set;

for i=1:length(m)

           % Get the list of all nodes in the graph
        V_list = S.get_element(V.get_idx());
        
        
       % Compute the distance from v_new to his parent
        dist = pdist2(S.container(S.container(m(i)).parent_idx).state, S.container(m(i)).state);
        S.container(m(i)).cost_from_parent=dist;
        
        updateChildCosts(m(i),S,V);

end
end



