function [val, gradient] = objFunc(x)

    global param evalCount;

    evalCount = evalCount + 1
    
    % Parameters: x = [tau1, tau2, initial theta1, initial theta2, time duration]    
    param.tau1 = x(1);
    param.tau2 = x(2);
    param.th1_0 = x(3);
    param.th2_0 = x(4);
    param.time_total = x(5);

    % Simulate throw using chosen parameters
    [tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = openLoopSim();

    % Check for joint limit violations
    if max(theta1_OL) > pi || max(theta2_OL) > pi ...
        || min(theta1_OL) < 0 || min(theta2_OL) < 0
        val = 10000;
    else

        % Put theta and theta dot into matrix form for
        % forwardKinematics function
        thetaMat = [theta1_OL, theta2_OL];
        thetaDotMat = [theta1dot_OL, theta2dot_OL];

        [handX, handY, handXdot, handYdot] =...
            forwardKinematics(thetaMat, thetaDotMat, param);

        % Obtain maximum velocity and its index
        [maxVel, maxVelIndex] = max(sqrt(handXdot.^2 + handYdot.^2));
        
        % Obtain frisbee direction at time of max. velocity
        maxVelAng = atan2(handYdot(maxVelIndex),handXdot(maxVelIndex));
        
        % Obtain frisbee spin at time of max. velocity
        maxVelSpin = 0; % TBD

        % Compute cost function value based on weighted RMSE of desired
        % frisbee characteristics
        wVel = 1;
        wAng = 1;
        wSpin = 0;
        val = sqrt(wVel*(maxVel-param.velGoal)^2+...
            wAng*(maxVelAng-param.angGoal)^2+...
            wSpin*(maxVelSpin-param.spinGoal)^2)/...
            (sqrt(wVel)+sqrt(wAng)+sqrt(wSpin));
        
    end
        
    end