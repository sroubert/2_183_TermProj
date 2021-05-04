function [x,xvel,xacc,t]=getMinJerk(X0,XF,T,dt)
    %takes in initial pos x0, final pos xf, and total time T 
    %outputs time trajectory x which is N by 2 
    %N is T/dt;
    del_x=XF(1)-X0(1);
    del_y=XF(2)-X0(2);
    t=0:dt:T;
    
    x=zeros(length(t),2);
    xvel=zeros(length(t),2);
    xacc=zeros(length(t),2);
    
    for i=1:length(t)
        tt=t(i);
        
        x(i,:)=[pos(X0(1),del_x,T,tt),pos(X0(2),del_y,T,tt)];
        xvel(i,:)=[vel(del_x,T,tt),vel(del_y,T,tt)];
        xacc(i,:)=[acc(del_x,T,tt),acc(del_y,T,tt)];
    end
end

function [x]=pos(x0,dx,T,t)
    x=x0+dx*(6*(t/T)^5-15*(t/T)^4+10*(t/T)^3);
end

function [xvel]=vel(dx,T,t)
    xvel=dx*(30*(t/T)^4 - 60*(t/T)^3 + 30*(t/T)^2);
end

function [xacc]=acc(dx,T,t)
    xacc=dx*(120*(t/T)^3 - 180*(t/T)^2 +60*(t/T));
end