% Takes a maximum torque magnitude, current time, total move duration, type
% of torque input shape, and number of degrees of freedom. Returns output
% torque at the current time in a 1D array the same length as the number of
% DOFs.

function tau = torqueInput(tauMag,time,duration,type,DOF)
   
    tau = zeros(1,DOF);
    
    if type == "equal and opposite"
        if time < duration/2
            for i = 1:DOF
               tau(i) = tauMag(i);
            end
        else
            for i = 1:DOF
               tau(i) = -tauMag(i);
            end
        end
    end
    
    if type == "constant"
       for i = 1:DOF
           tau(i) = tauMag(i);
        end
    end
    
%     if type == "bell curve"
%        for i = 1:DOF
%            % your code goes here
%         end
%     end
    
end