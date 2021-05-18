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
        th3_initial=pi/4; %relative coords
        l3=params.l3;
        delta_th3=params.delta_th3;
        D=params.time_total;
        th3=zeros(length(t),1);
        theta_min_jerk=@(t,A) 6*A*(t/D)^5-15*A*(t/D)^4 +10*A*(t/D)^3;
        
        for i=1:length(t)%same approach as 2 dof
            x=xy(i,1);
            y=xy(i,2);
            l3=sqrt(x^2+y^2);
            th1(i)=atan2(y,x) - acos((l1^2+l3^2-l2^2)/(2*l1*l3));
            th2(i)=th1(i)+pi-acos((l1^2+l2^2-l3^2)/(2*l1*l2));
            
            %now do th3
            tt=t(i);

            th3(i)=th2(i)+th3_initial+theta_min_jerk(tt,delta_th3);

        end
        thetas=[th1,th2,th3];
    end

end