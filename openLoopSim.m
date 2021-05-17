function [tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = openLoopSim()
%assumes p is a structure containing the parameters of mass, 
%centroidal inertias, lengths, and initial states, ie angles and omegas
%for this also has constant torques

global param control;

time_total = param.time_total;

framessec=60; 
tspan=linspace(0,time_total,time_total*framessec);

%choose type of torque input and number of DOFs for torque input function
type = "equal and opposite";
% type = "constant";
%DOF = 2;

%unpack initial states from param
%th1_0 = param.th1_0; th2_0 = param.th2_0;

th1dot_0 = param.th1dot_0; th2dot_0 = param.th2dot_0;


%integration
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);

%initialize arrays to hold tau values to check function is working
tau1Array = zeros(1,1);
tau2Array = zeros(1,1);

if control == "hand position"
    x0=[param.xi, param.yi];
    xF=[param.xf, param.yf];
    dt=time_total/100;
    [x,v,a,t]=getMinJerk(x0,xF,time_total,dt); %min jerk trj
    param.dt=dt;
    param.x=x;
    param.v=v;
    param.t=t;
    param.thd=getThetasFromXY(x,v,a,t,param); %inverse kinematics + other for 3 dof
    
    th1_0=param.thd(1,1);
    th2_0=param.thd(1,2);
    if param.dof==3
       th3_0=param.thd(1,3); 
       th3dot_0=0
    end
end
if param.dof==2
state0 = [th1_0, th2_0, th1dot_0, th2dot_0];
elseif param.dof==3
    state0=[th1_0 th2_0 th3_0 th1dot_0 th2dot_0 th3dot_0];
end

[tarray, statearray] = ode45(@RHS, param.t, state0, options);

theta1_OL = statearray(:,1); theta2_OL = statearray(:,2);
theta1dot_OL = statearray(:,3); theta2dot_OL = statearray(:,4);

%remove duplicate time values from tau arrays and sort in ascending order
%tau1Array = tau1Array(2:end,:);
%tau2Array = tau2Array(2:end,:);
%tau1Array = unique(tau1Array,'rows');
%tau2Array = unique(tau2Array,'rows');

%interpolate torque arrays to be same length as time vector
%tau1Norm = interp1(tau1Array(:,1),tau1Array(:,2),tspan);
%tau2Norm = interp1(tau2Array(:,1),tau2Array(:,2),tspan);

%plot joint torques in their own window
% figure();
% subplot(2,1,1)
% plot(tarray,tau1Norm,'LineWidth',2);
% xlabel('Time (s)')
% ylabel('Joint 1 Torque (N-m)')
% subplot(2,1,2)
% plot(tarray,tau2Norm,'LineWidth',2);
% xlabel('Time (s)')
% ylabel('Joint 2 Torque (N-m)')
% sgtitle('Joint Torques')
param.tau=[tau1Array,tau2Array];

%integration function using Lagrange
    function stateArmDot = RHS(t,z) %xdot = Ax + Bu
        %unpacking struct
        l1 = param.l1; l2 = param.l2; m1 = param.m1; m2 = param.m2;

        Ic1 = param.Ic1; Ic2 = param.Ic2; 

        %if param.dof == 2
            %tauMag = [param.tau1, param.tau2];
        %elseif param.dof == 3
            %tauMag = [param.tau1, param.tau2, param.tau3];
        %end
        
        %minJerkParams = [param.xi, param.yi, param.xf, param.yf];


        %tau = torqueInput(tauMag,minJerkParams,t,time_total,type,DOF); two
        %impulses
        tau=torqueFromImpedance(z,t,param);
            
        tau1 = tau(1);
        tau2 = tau(2);
        
        % check that torques are what we wanted
        tau1Array(end+1,:) = [tau1];
        tau2Array(end+1,:) = [tau2];
        
        %unpacking state
        
        th1 = z(1); th2 = z(2); th1dot = z(3); th2dot = z(4);
        
        %EOM from ddthetaXLag files
        th1dotdot = ddtheta1Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        th2dotdot = ddtheta2Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        
        stateArmDot = [th1dot; th2dot; th1dotdot; th2dotdot];
    end

end