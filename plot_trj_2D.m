function [] =plot_trj_2D(dt,th1s,th2s,params)
    %Assumes constant dt
    h=figure;
    axes 
    xlim([-1,2])
    
    for k =1:length(th1s)
        th1=th1s(k);
        th2=th2s(k);
        [armx,army]=get_arm(th1,th2,params);
        [discx,discy]=get_disc(th1,th2,params);
        tmp=plot(armx,army,'k','LineWidth',2);
        hold on
        tmp2=plot(discx,discy,'r','LineWidth',2);
        axis equal
        drawnow
        pause(dt)
        
        if k~=length(th1s)
            delete(tmp)
            delete(tmp2)
        end
    end
end

function [x,y]=get_arm(th1,th2,params)
    %CURRENTLY ASSUMES ABSOLUTE ANGLES
    l1=params.l1;
    l2=params.l2;
    x=[0;l1*cos(th1);l1*cos(th1)+l2*cos(th2)];
    y=[0;l1*sin(th1);l1*sin(th1)+l2*sin(th2)];
end

function [x,y]=get_disc(th1,th2,params);
    l1=params.l1;
    l2=params.l2;
    r=0.1; %radius of disc in m
    xc=l1*cos(th1)+l2*cos(th2) + r*cos(th2+pi/2); %hand position plus offset
    yc=l1*sin(th1)+l2*sin(th2) + r*sin(th2+pi/2);

    num=100;

    for i=1:num+1
        th=0+(i-1)*2*pi/num;
        x(i)=xc+r*cos(th);
        y(i)=yc+r*sin(th);
    end
end