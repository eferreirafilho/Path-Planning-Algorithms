%Sample inside an ellipse that connect q_start and q_goal

function xrand = sample(xstart,xgoal,cmax,map_grid,S)

rand_aux2=1;
while rand_aux2==1
if cmax<inf
    
    
    cmin=sqrt((xgoal(2)-xstart(2))^2 + (xgoal(1)-xstart(1))^2);

    
    xcentre(1,1)=(xgoal(1)+xstart(1))/2;
    xcentre(2,1)=(xgoal(2)+xstart(2))/2;
    C =rotationtoworldframe(xstart,xgoal);
    r1=cmax/2;
    
    r2=sqrt(cmax^2 - cmin^2)/2;
    L=diag([r1 r2]);
    xball=sampleunitball;
    xrand=C*L*xball+xcentre;
    xrand=xrand';

   
else
    
    inc_factor=size(map_grid,1);
    
    
    %Generate random node
    xrand_aux=1;
    while (xrand_aux==1)
            xrand=(inc_factor-1)*[rand, rand] +1;
            xrand=round(xrand);
            xrand_aux=sample_free(xrand(1),xrand(2),map_grid);
    end
    
    
end


%Check if new point already exists
rand_aux2=0;
for i=1:length(S.container)
if xrand==S.container(i).state
    rand_aux2=1;
      
end
end


end