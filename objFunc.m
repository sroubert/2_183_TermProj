function [val, gradient] = objFunc(x)

    global param control evalCount;

    evalCount = evalCount + 1
    
    if control == "torque"
        % Torque control
        % Parameters: x = [tau1, tau2, initial theta1, initial theta2, time duration]    
        param.tau1 = x(1);
        param.tau2 = x(2);
        param.th1_0 = x(3);
        param.th2_0 = x(4);
        param.time_total = x(5);
    
    elseif control == "hand position"
        % Cartesian position control
        % Parameters: x = [xi, yi, xf, yf, time duration]
        param.xi = x(1);
        param.yi = x(2);
        param.xf = x(3);
        param.yf = x(4);
        param.time_total = x(5);
        if param.dof==3
            param.delta_th3=x(6); %distance traveled of wrist
            param.hand_ti=x(7); %start time of wrist motion
            param.hand_T=x(8); %total time for movement
        end
        
        % Check for start or end position outside of workspace
        if (sqrt(param.xi^2+param.yi^2) > param.l1+param.l2) || (sqrt(param.xf^2+param.yf^2) > param.l1+param.l2)
            disp('Outside of workspace')
            param.xi
            param.yi
            param.xf
            param.yf
            val = 10000;
            return
        end
    end
        

    % Simulate throw using chosen parameters
    [tarray, thetaMat, thetaDotMat] = openLoopSim();

    th1=thetaMat(:,1);
    th2=thetaMat(:,2);
    th1dot=thetaDotMat(:,1);
    th2dot=thetaDotMat(:,2);
    if param.dof==3
       th3=thetaMat(:,3);
       th3dot=thetaDotMat(:,3);
    end
    % Check for joint limit violations
    if max(th1) > pi || max(th2-th1) > pi ...
        || min(th1) < 0 || min(th2-th1) < 0
        val = 10000;
        return
    elseif param.dof==3 && (max(th3-th2) > pi/2 || min(th3-th2) < -pi/2)    % Wrist allowed to go +/- 90 deg from forearm
        val = 10000;
        return
    else

        % Put theta and theta dot into matrix form for
        % forwardKinematics function
%         if param.dof==2
%             thetaMat = [theta1_OL, theta2_OL];
%             thetaDotMat = [theta1dot_OL, theta2dot_OL];
%         elseif param.dof==3
%             thetaMat = [theta1_OL, theta2_OL, theta3_OL];
%             thetaDotMat = [theta1dot_OL, theta2dot_OL, theta3dot_OL];
%         end

        [handX, handY, handXdot, handYdot] =...
            forwardKinematics(thetaMat, thetaDotMat, param);
        if (any(imag(handXdot)~=0) || any(imag(handYdot)~=0))
            val=10000;
            return;
        end
        % Obtain maximum velocity and its index
        [maxVel, maxVelIndex] = max(sqrt(handXdot.^2 + handYdot.^2));
        
        % Obtain frisbee direction at time of max. velocity
        maxVelAng = atan2(handYdot(maxVelIndex),handXdot(maxVelIndex));
        
        % Obtain frisbee spin at time of max. velocity
        if param.dof==2
            maxVelSpin =  th2dot(maxVelIndex);
        elseif param.dof==3
            maxVelSpin = th3dot(maxVelIndex);
        end

        % Compute cost function value based on weighted RMSE of desired
        % frisbee characteristics
        if contains(param.objective, "V")   % frisbee velocity
            wVel = 1;
        else
            wVel = 0;
        end
        if contains(param.objective, "A")   % frisbee release angle
            wAng = 0.1;
        else
            wAng = 0;
        end
        if contains(param.objective, "S")   % frisbee spin
            wSpin = 1;
        else
            wSpin = 0;
        end
        
        val = sqrt(wVel*(maxVel-param.velGoal)^2+...
            wAng*(maxVelAng-param.angGoal)^2+...
            wSpin*(maxVelSpin-param.spinGoal)^2)/...
            (sqrt(wVel)+sqrt(wAng)+sqrt(wSpin));
        
    end
        
    end