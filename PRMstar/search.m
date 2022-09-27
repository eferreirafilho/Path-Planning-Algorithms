% Essa fun��o calcula a �rvore BFS do grafo S a partir da raiz v
% O retorno CC contem todos os n�s de S mas com o parent_idx alterado.
% Aten��o: A ordem de armazenamento em CC n�o � igual aquela em S!
% Sintaxe: CC = search(v, S)
function CC = search(v, S)
    
    CC = container_set(vertex.empty()); % Esse container retornar� os n�s modificados com seus parents
    C=[];                               % Conjunto C -> FIFO
    
    % coloca v em O
    O=[v v]; % Conjunto O. Primeira coluna � o v�rtice e segunda o pai do v�rtice
        
    % enquanto a fila n�o est� vazia
    while length(O(:,1))>0
        % remove u de O -> FIFO implica remo��o do primeiro da fila
        u=O(1,1);  % u
        pu=O(1,2); % parent of u
        O(1,:)=[]; % Elimina a posi��o 1 -> Matlab cuida do resto!
        
        % Se u n�o estiver em C coloque u em C
        if length(find(C==u))==0
            % coloque u em C
            C(end+1)=u;
            n = S.container(u); % Copia o n� U de S para CC modificando seu parent
            n.parent_idx=pu;
            CC.add_element(n);
            
            % coloque os vizinhos de u que n�o est�o em C em O
            for i=1:length(S.container(u).children_set)
                if length(find(C==S.container(u).children_set(i)))==0
                    O(end+1,1)=S.container(u).children_set(i);
                    O(end,2)=u; % Indica que o n� foir para O por ser vizinho de u
                end
            end            
        end 
    end