
global control evalCount param;
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


param.K=[29.5,14.3;14.3,39.3]*2;
if param.dof==3
   param.K=[param.K,[0;0];  0 0 30];
   
   param.l2=param.l2-param.l3; %updating forearm mass, lengths and inertias
   param.m2=param.m2-param.m3; %based on 3 dof with distribution to wrist
   param.I2=1/12*param.m2*param.l2^2;
end

param.B=param.K*.1;
%% optimization

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

%% simulate

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

%% saving outputs of optimization

currentFolder = pwd; %string of current folder
analysisFolder = fullfile(pwd, 'analysis');

fileName = strcat(string(param.dof),'DOF','_',...
    'frisOr',string(param.thFrisOrient),...
    '_','obj',param.objective, ...
    '_','vel',string(param.velGoal),'_','ang',string(param.angGoal), ...
    '_','spin',string(param.spinGoal), '.mat');

fileToSave = fullfile(analysisFolder,fileName);

save(fileToSave,'param')

simOutput = [maxVel, maxVelAng, maxVelSpin];

fileNameOut = fullfile(analysisFolder,strcat('OUTPUT',fileName));

save(fileNameOut,'simOutput')

