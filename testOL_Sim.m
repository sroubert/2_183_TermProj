close all;
clear all;

global param control evalCount;
evalCount = 0;      % Count how many optimization evaluations were run

param.maxEvals=100;

param.algorithm = NLOPT_GN_DIRECT_L;
% param.algorithm = NLOPT_GN_CRS2_LM;

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

param.l3=.06;
param.m3=.24; %estimation based on l3/l2*m2
param.Ic3=1/12*param.m3*param.l3^2;

param.mfris=.175; %grams
param.rfris=.274/2; %m
param.Ifris=.6*param.mfris*param.rfris^2; %estimation of I. mostly a disc, but some mass closer to ring;

param.thFrisOrient=pi;

% Set goals for frisbee behavior (linear velocity, direction, and spin)
param.velGoal = 14;          % Desired maximum frisbee linear velocity
param.angGoal = pi/4;       % Desired frisbee direction at max. velocity
param.spinGoal = -50;         % Desired frisbee spin at max. velocity SHOULD BE NEGATIVE  

% Choose what error optimization will try to minimize - velocity, angle,
% spin, or some combination. Used by objFunc.m
%param.objective = "V";
% param.objective = "A";
% param.objective = "S";
% param.objective = "VA";
 param.objective = "VS";
% param.objective = "AS";
%  param.objective = "VAS";
 

% Choose type of control (uncomment desired choice)
%control = "torque";
 control = "hand position";

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

param.tau_1_max=50;
param.tau_2_max=30;
param.tau_3_max=20;

param.dof=3;

param.K=[29.5,14.3;14.3,39.3]*2;
if param.dof==3
   param.K=[param.K,[0;0];  0 0 30];
end

param.B=param.K*.1;

% Choose whether to run an optimization or manually select values
runOptimization = true;

if runOptimization
    % Runs optimization with given physical parameters and desired behavior.
    % Assigns optimized throw parameters to global param structure.
    % Returns minimum error value.
    % Optimization bounds and settings are set in optimize.m function.
    fmin = optimize();

    % Display results of optimization
    param       % Optimal parameters
    fmin        % Minimum objective function value
    evalCount   % Number of optimization evaluations
else
    % Manually assign parameters for simulation
    if control == "torque"
        param.tau1 = 0.5;
        param.tau2 = 0.5;
        param.th1_0 = 0;
        param.th2_0 = 0;
        param.time_total = 0.5;
    elseif control == "hand position"
        param.xi = -0.5;
        param.yi = 0;
        param.xf = 0.15;
        param.yf = 0.3;
        param.time_total = 0.5;
    end
end

% Simulate trajectory using optimal parameters
[tarray, thetaMat, thetaDotMat] = ...
    openLoopSim();

theta1_OL=thetaMat(:,1);
theta2_OL=thetaMat(:,2);
theta1dot_OL=thetaDotMat(:,1);
theta2dot_OL=thetaDotMat(:,2);
if param.dof==3
   theta3_OL=thetaMat(:,3);
   theta3dot_OL=thetaDotMat(:,3);
end

% Convert trajectory to Cartesian space
[x, y, xdot, ydot] = forwardKinematics(thetaMat, thetaDotMat, param);

velMag = sqrt(xdot.^2 + ydot.^2);
velAng = atan2(ydot,xdot);

% Return achieved frisbee behavior
[maxVel, maxVelIndex] = max(velMag)
maxVelAng = velAng(maxVelIndex)
if param.dof==2
    maxVelSpin =  theta2dot_OL(maxVelIndex)
elseif param.dof==3
    maxVelSpin = theta3dot_OL(maxVelIndex)
end

% Display optimal trajectory in joint angles
figure
subplot(2,2,1)
plot(tarray,rad2deg(theta1_OL))
hold on
plot(param.t,rad2deg(param.thd(:,1)),'r--')
title('theta_{1}')
xlabel('time (sec)')
ylabel('theta1 (deg)')

subplot(2,2,2)
plot(tarray,rad2deg(theta2_OL))
hold on
plot(param.t,rad2deg(param.thd(:,2)),'r--')
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
hold on
plot(param.t,param.x(:,1),'r--')
title('Hand x-coordinates')
xlabel('time (sec)')
ylabel('x (m)')

subplot(3,2,2)
plot(tarray,y)
hold on
plot(param.t,param.x(:,2),'r--')
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

% Display optimal trajectory on xy-plane
figure();
plot(x,y,'LineWidth',2)
hold on;
plot(x(1),y(1),'Marker','o','Color','green','LineWidth',1)          % Green circle at start point
plot(x(end),y(end),'Marker','diamond','Color','red','LineWidth',1)  % Red diamond at stop point
plot(x(maxVelIndex),y(maxVelIndex),'Marker','*','Color','black')     % Cyan asterisk at max vel point
% Add arrow showing frisbee max velocity & direction to plot
arrowLen = 0.5*maxVel/param.velGoal;
quiver(x(maxVelIndex),y(maxVelIndex),...
    arrowLen*cos(maxVelAng),arrowLen*sin(maxVelAng),...
    'Color','cyan','LineWidth',1)
% Set figure limits
margin = 0.1;
xlim1 = -param.l1-param.l2-margin;
xlim2 = param.l1+param.l2+margin;
ylim1 = -margin;
ylim2 = param.l1+param.l2+margin;
xlim([xlim1,xlim2])
ylim([ylim1, ylim2])
title('Hand Cartesian Coordinates')
xlabel('x (m)')
ylabel('y (m)')

plot_trj_2D(param.dt,theta1_OL,theta2_OL,param)