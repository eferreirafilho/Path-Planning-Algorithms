%Return the idx of the nearest node

function [Xnearest]=nearest(S,V,Xrand)
    
    
   % Get the list of all nodes in the graph
   V_list = S.get_element(V.get_idx());
        
   %Algorithm 1 - PRM - line 4
   % Compute de distance from v_new to all nodes of the graph
   [dist I] = pdist2(cell2mat({V_list.state}'), Xrand,'euclidean','Smallest',1);
   
   Xnearest =  I;%sorted_idx
   

end