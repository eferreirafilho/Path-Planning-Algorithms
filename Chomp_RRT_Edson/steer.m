%Return a point in the direction of Xrand
%Size regulated by step_size

function [Xnew]=steer(Xnearest,Xrand,step_size)

        %Line between Xrand and Xnearest
        vec_steer=(Xrand-Xnearest);
        %Normalized line
        vec_steer_norm=vec_steer/norm(vec_steer);
        
        %Step in the direction of Xrand
        Xnew = round(Xnearest + step_size*vec_steer_norm);
     
        %Check if points are the same
        if Xnearest==Xrand
            Xnew=Xrand;
        end
     
        
end