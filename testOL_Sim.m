global param;

param.m1 = 1;
param.m2 = 2;
param.Ic1 = 1;
param.Ic2 = 2;

param.l1 = 1;
param.l2 = 2;
param.th1dot_0 = 0;
param.th2dot_0 = 0;
param.maxVelGoal = 10;

% Choose whether to run an optimization or manually select values
runOptimization = false;

if runOptimization
    % Runs optimization with given physical parameters and desired behavior.
    % Assigns optimized throw parameters to global param structure.
    % Returns minimum error value.
    % Optimization bounds and settings are set in optimize.m function.
    fmin = optimize();

    % Display optimal parameters
    param
    fmin
else
    % Manually assign parameters for simulation
    param.tau1 = 5;
    param.tau2 = 5;
    param.th1_0 = 0;
    param.th2_0 = 0;
    param.time_total = 5;
end

% Simulate trajectory using optimal parameters
[tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = ...
    openLoopSim();

% Put joint angles and velocities in arrays
thetaMat = [theta1_OL, theta2_OL];
thetaDotMat = [theta1dot_OL, theta2dot_OL];

% Convert trajectory to Cartesian space
[x, y, xdot, ydot] = forwardKinematics(thetaMat, thetaDotMat, param);

velMag = sqrt(xdot.^2 + ydot.^2);
velAng = atan2(ydot,xdot);

% Display optimal trajectory in joint angles
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

% Display optimal trajectory in Cartesian coordinates
figure
subplot(3,2,1)
plot(tarray,x)
title('Hand x-coordinates')
xlabel('time (sec)')
ylabel('x (m)')

subplot(3,2,2)
plot(tarray,y)
title('Hand y-coordinates')
xlabel('time (sec)')
ylabel('y (m)')

subplot(3,2,3)
plot(tarray,xdot)
title('Hand x-velocity (m/s)')
xlabel('time (sec)')
ylabel('v_{x} (m/s)')

subplot(3,2,4)
plot(tarray,ydot)
title('Hand y-velocity (m/s)')
xlabel('time (sec)')
ylabel('v_{y} (m/s)')

subplot(3,2,5)
plot(tarray,velMag)
title('Tangential velocity (m/s)')
xlabel('time (sec)')
ylabel('v (m/s)')

subplot(3,2,6)
polarplot(velAng,velMag)
title('Velocity Trajectory')
