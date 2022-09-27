%CHOMP function
%Created by Guilherme Pereira 03/11/2016
%Modified by Edson Bernardes - 05/11/2016

function [smooth_traj] = chomp(traji,brushfire_map)


traj1=[];
traj2=[];
for i=1:length(traji)-1
    traj1=[traj1 linspace(traji(1,i), traji(1,i+1), 5)];
    traj2=[traj2 linspace(traji(2,i), traji(2,i+1), 5)];
end
traj=[traj1;traj2];
plot(traj(1,:), traj(2,:))
hold on
niter=5;%Number of iterations
mu = 0.5;%Smoothness
alpha = 0.02;%Avoid obstacles
epsilon = size(brushfire_map,1)/50;%Distance of obstacle detection

for n=1:niter
    for i=2:length(traj)-1
        qi=traj(:,i);
        qimais1=traj(:,i+1);
        qimenos1=traj(:,i-1);
        
        %Smoothness gradient
        gradsmooth=qimais1+qimenos1-2*qi;
        
        dq=qi-qimenos1;
        ddq=qimais1-qimenos1;
        
        %Get distance to closer obstacle and gradient
        [d, gradc]=query_brushfire(brushfire_map, round(qi)');

        if d<0
            c=-d+0.5*epsilon;
        else if d>epsilon
                c=0;
            else
                c=1/(2*epsilon)*(d-epsilon)^2;
            end
        end
          
        %Obstacle Gradient
        gradobst=norm(dq)*(eye(2)-dq*dq')*gradc-c*(norm(dq)^-2*(eye(2)-dq*dq')*ddq);
        

       %Total Gradient
        grad=gradsmooth+alpha*gradobst;
                
        %If gradient grown too much and became degenerated
        if sum(isnan(grad))>=1
            disp('Warning -> Degenerated gradient. Try smaller alpha')
            grad=[0;0];
        end

        traj(:,i)=traj(:,i)+mu*grad;
        
    end
    
end
smooth_traj=traj;
