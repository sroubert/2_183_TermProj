
param.m1 = 1;
param.m2 = 2;
param.Ic1 = 1;
param.Ic2 = 2;

param.l1 = 1;
param.l2 = 2;
param.tau1 = 1;
param.tau2 = 5;
param.th1_0 = 0;
param.th2_0 = pi/4;
param.th1dot_0 = 0;
param.th2dot_0 = 0;

time_total = 5; %seconds

[tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = ...
    openLoopSim(param, time_total);

figure
subplot(2,2,1)
plot(tarray,theta1_OL)
title('theta1')
xlabel('time (sec)')

subplot(2,2,2)
plot(tarray,theta2_OL)
title('theta2')
xlabel('time (sec)')

subplot(2,2,3)
plot(tarray,theta1dot_OL)
title('theta1dot')
xlabel('time (sec)')

subplot(2,2,4)
plot(tarray,theta2dot_OL)
title('theta2dot')
xlabel('time (sec)')
