% forwardKinematics

function [x, y, xdot, ydot] = forwardKinematics(thetaMat, thetaDotMat, param)
    
%     % DEBUG
%     szDOF = length(thetaMat(:,1
    
    % 2 DOF
    if length(thetaMat(1,:)) == 2   
        
        theta1 = thetaMat(:,1);
        theta2 = thetaMat(:,2);
        theta1dot = thetaDotMat(:,1);
        theta2dot = thetaDotMat(:,2);
        
        x = param.l1*cos(theta1) + param.l2*cos(theta2);
        y = param.l1*sin(theta1) + param.l2*sin(theta2);
        
        xdot = -param.l1*sin(theta1).*theta1dot - param.l2*sin(theta2).*theta2dot;
        ydot = param.l1*cos(theta1).*theta1dot + param.l2*cos(theta2).*theta2dot;
        
    % 3 DOF
    elseif length(thetaMat(1,:)) == 3
        
        theta1 = thetaMat(:,1);
        theta2 = thetaMat(:,2);
        theta3 = thetaMat(:,3);
        theta1dot = thetaDotMat(:,1);
        theta2dot = thetaDotMat(:,2);
        theta3dot = thetaDotMat(:,3);
        
        x = param.l1*cos(theta1) + param.l2*cos(theta2) + param.l3*cos(theta3);
        y = param.l1*sin(theta1) + param.l2*sin(theta2) + param.l3*sin(theta3);
        
        xdot = -param.l1*sin(theta1).*theta1dot - param.l2*sin(theta2).*theta2dot...
            - param.l3*sin(theta3).*theta3dot;
        ydot = param.l1*cos(theta1).*theta1dot + param.l2*cos(theta2).*theta2dot...
            + param.l3*cos(theta3).*theta3dot;
        
    end
