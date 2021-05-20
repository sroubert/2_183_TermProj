function [] = save_trj_3D(dt,ths,params,file_name)
    h=figure;
    axes 
    axis equal
    xlim([-.8,0.8])
    ylim([-.2,0.8])
    th1s=ths(:,1);
    th2s=ths(:,2);
    th3s=ths(:,3);
    hold on
    for k =1:length(th1s)
        th1=th1s(k);
        th2=th2s(k);
        th3=th3s(k);
        [armx,army]=get_arm(th1,th2,th3,params);
        [discx,discy]=get_disc(th1,th2,th3,params);
        tmp=plot(armx,army,'k','LineWidth',2);
        tmp2=plot(discx,discy,'r','LineWidth',2);

        M(k)=getframe(h);
        pause(dt)
        delete(tmp)
        delete(tmp2)
    end
    hold off
    v=VideoWriter(file_name,'MPEG-4');
    v.FrameRate = 1/dt;
    open(v)
    writeVideo(v,M)
    %v
    close(v)
end

function [x,y]=get_arm(th1,th2,th3,params)
    %CURRENTLY ASSUMES ABSOLUTE ANGLES
    l1=params.l1;
    l2=params.l2;
    l3=params.l3;
    x=[0;l1*cos(th1);l1*cos(th1)+l2*cos(th2);l1*cos(th1)+l2*cos(th2)+l3*cos(th3)];
    y=[0;l1*sin(th1);l1*sin(th1)+l2*sin(th2);l1*sin(th1)+l2*sin(th2)+l3*sin(th3)];
end

function [x,y]=get_disc(th1,th2,th3,params)
    l1=params.l1;
    l2=params.l2;
    l3=params.l3;
    r=0.137; %m
    xc=l1*cos(th1)+l2*cos(th2)+l3*cos(th3) + r*cos(th3+params.thFrisOrient); %hand position plus offset
    yc=l1*sin(th1)+l2*sin(th2)+l3*sin(th3) + r*sin(th3+params.thFrisOrient);

    num=100;

    for i=1:num+1
        th=0+(i-1)*2*pi/num;
        x(i)=xc+r*cos(th);
        y(i)=yc+r*sin(th);
    end
end