close all;
clear all;

global param control evalCount;
evalCount = 0;      % Count how many optimization evaluations were run

% Using Neville's estimates for limb segment lengths & masses (from
% "Multi-Joint Inertial Dynamics" reading
param.m1 = 2.52;
param.m2 = 1.30;
param.l1 = 0.33;
param.l2 = 0.32;
param.Ic1 = 1/12*param.m1*param.l1^2;   % Define moments of inertia based on arm as thin rod
param.Ic2 = 1/12*param.m2*param.l2^2;
param.th1dot_0 = 0;
param.th2dot_0 = 0;

% Set goals for frisbee behavior (linear velocity, direction, and spin)
param.velGoal = 5;          % Desired maximum frisbee linear velocity
param.angGoal = pi/2;       % Desired frisbee direction at max. velocity
param.spinGoal = 0;         % Desired frisbee spin at max. velocity

% Choose type of control (uncomment desired choice)
control = "torque";
% control = "hand space";

% Intitialize parameters that will be used by objective function to zero
param.tau1 = 0;
param.tau2 = 0;
param.th1_0 = 0;
param.th2_0 = 0;  
param.xi = 0;
param.yi = 0;
param.xf = 0;
param.yf = 0;
param.time_total = 0;

% Choose whether to run an optimization or manually select values
runOptimization = true;

if runOptimization
    % Runs optimization with given physical parameters and desired behavior.
    % Assigns optimized throw parameters to global param structure.
    % Returns minimum error value.
    % Optimization bounds and settings are set in optimize.m function.
    fmin = optimize();

    % Display results of optimization
    param   % Optimal parameters
    fmin    % Minimum objective function value
    evalCount   % Number of optimization evaluations
else
    % Manually assign parameters for simulation
    param.tau1 = 0.5;
    param.tau2 = 0.5;
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

% Return achieved frisbee behavior
[maxVel, maxVelIndex] = max(velMag)
maxVelAng = velAng(maxVelIndex)

% Display optimal trajectory in joint angles
figure
subplot(2,2,1)
plot(tarray,rad2deg(theta1_OL))
title('theta_{1}')
xlabel('time (sec)')
ylabel('theta1 (deg)')

subplot(2,2,2)
plot(tarray,rad2deg(theta2_OL))
title('theta_{2}')
xlabel('time (sec)')
ylabel('theta2 (deg)')

subplot(2,2,3)
plot(tarray,rad2deg(theta1dot_OL))
title('theta_{1} dot')
xlabel('time (sec)')
ylabel('theta1dot (deg/s)')


subplot(2,2,4)
plot(tarray,rad2deg(theta2dot_OL))
title('theta_{2} dot')
xlabel('time (sec)')
ylabel('theta2dot (deg/s)')

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
