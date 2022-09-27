% Essa função calcula a árvore BFS do grafo S a partir da raiz v
% O retorno CC contem todos os nós de S mas com o parent_idx alterado.
% Atenção: A ordem de armazenamento em CC não é igual aquela em S!
% Sintaxe: CC = search(v, S)
function CC = search(v, S)
    
    CC = container_set(vertex.empty()); % Esse container retornará os nós modificados com seus parents
    C=[];                               % Conjunto C -> FIFO
    
    % coloca v em O
    O=[v v]; % Conjunto O. Primeira coluna é o vértice e segunda o pai do vértice
        
    % enquanto a fila não está vazia
    while length(O(:,1))>0
        % remove u de O -> FIFO implica remoção do primeiro da fila
        u=O(1,1);  % u
        pu=O(1,2); % parent of u
        O(1,:)=[]; % Elimina a posição 1 -> Matlab cuida do resto!
        
        % Se u não estiver em C coloque u em C
        if length(find(C==u))==0
            % coloque u em C
            C(end+1)=u;
            n = S.container(u); % Copia o nó U de S para CC modificando seu parent
            n.parent_idx=pu;
            CC.add_element(n);
            
            % coloque os vizinhos de u que não estão em C em O
            for i=1:length(S.container(u).children_set)
                if length(find(C==S.container(u).children_set(i)))==0
                    O(end+1,1)=S.container(u).children_set(i);
                    O(end,2)=u; % Indica que o nó foir para O por ser vizinho de u
                end
            end            
        end 
    end