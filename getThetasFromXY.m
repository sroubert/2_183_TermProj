function [thetas]=getThetasFromXY(xy,v,a,t,params)
    %Only 2 DOF to start, maybe expand
    %what kind of sub-optimization? min energy, 
    th1=zeros(length(t),1);
    th2=zeros(length(t),1);
    l1=params.l1;
    l2=params.l2;
    DOF=params.dof;
    %type=params.type; min energy etc for 3 dof
    if DOF==2
        %inverse kinematics
        for i=1:length(t)
            x=xy(i,1);
            y=xy(i,2);
            l3=sqrt(x^2+y^2);
            th1(i)=atan2(y,x) - acos((l1^2+l3^2-l2^2)/(2*l1*l3));
            th2(i)=th1(i)+pi-acos((l1^2+l2^2-l3^2)/(2*l1*l2));
        end
        thetas=[th1,th2];
    end
    
    if DOF==3
        l3=params.l3;
        th3=zeros(length(t),1);
        %Sub-optimization :) 
        switch type
            case 'min-energy'
                for i =1:length(t)
                    W=M(i);%Mass Matrix to get energy (how to know mass matrix without knowing thetas?)
                    %or W=Ito get angular velocity norm
                    J=get_jacobian();%again, how to get jacobian when you don't know configuration
                    %J_w_plus=inv(W) * J'*inv(J*inv(W)*J')
                    %[th1(i);th2(i);th3(i)]=J_w_plus*xvel(i,:)';
                end
            case ''
            
            case ''
        end
        thetas=[th1,th2,th3];
    end

end