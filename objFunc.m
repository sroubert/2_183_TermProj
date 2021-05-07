function [val, gradient] = objFunc(x)
    
%         global goal m1 m2 Ic1 Ic2 l1 l2;
        global param;
        
%         Parameters: x = [tau1, tau2, initial theta1, initial theta2, time duration]
    
        param.tau1 = x(1);
        param.tau2 = x(2);
        param.th1_0 = x(3);
        param.th2_0 = x(4);
        param.time_total = x(5);

        [tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = openLoopSim();

        thetaMat = [theta1_OL, theta2_OL];
        thetaDotMat = [theta1dot_OL, theta2dot_OL];
        
        [handX, handY, handXdot, handYdot] = forwardKinematics(thetaMat, thetaDotMat, param);
        
        maxVel = max(sqrt(handXdot.^2 + handYdot.^2));
        
        val = sqrt((maxVel - param.maxVelGoal)^2);
        
    end