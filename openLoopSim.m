function [tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = openLoopSim(p, time_total)
%assumes p is a structure containing the parameters of mass, 
%centroidal inertias, lengths, and initial states, ie angles and omegas
%for this also has constant torques


framessec=60; 
tspan=linspace(0,time_total,time_total*framessec);

%choose type of torque input and number of DOFs for torque input function
% type = "equal and opposite";
type = "constant";
DOF = 2;

%unpack initial states from p
th1_0 = p.th1_0; th2_0 = p.th2_0;

th1dot_0 = p.th1dot_0; th2dot_0 = p.th2dot_0;

%initial state array

state0 = [th1_0, th2_0, th1dot_0, th2dot_0];
%integration
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);

%initialize arrays to hold tau values to check function is working
tau1Array = zeros(1,2);
tau2Array = zeros(1,2);

[tarray, statearray] = ode45(@RHS, tspan, state0, options, p);

theta1_OL = statearray(:,1); theta2_OL = statearray(:,2);
theta1dot_OL = statearray(:,3); theta2dot_OL = statearray(:,4);

%remove duplicate time values from tau arrays and sort in ascending order
tau1Array = tau1Array(2:end,:);
tau2Array = tau2Array(2:end,:);
tau1Array = unique(tau1Array,'rows');
tau2Array = unique(tau2Array,'rows');

%interpolate torque arrays to be same length as time vector
tau1Norm = interp1(tau1Array(:,1),tau1Array(:,2),tspan);
tau2Norm = interp1(tau2Array(:,1),tau2Array(:,2),tspan);

%plot joint torques in their own window
figure();
subplot(2,1,1)
plot(tarray,tau1Norm,'LineWidth',2);
xlabel('Time (s)')
ylabel('Joint 1 Torque (N-m)')
subplot(2,1,2)
plot(tarray,tau2Norm,'LineWidth',2);
xlabel('Time (s)')
ylabel('Joint 2 Torque (N-m)')
sgtitle('Joint Torques')

%integration function using Lagrange
    function stateArmDot = RHS(t,z,p) %xdot = Ax + Bu
       %unpacking struct
       l1 = p.l1; l2 = p.l2; m1 = p.m1; m2 = p.m2;

        Ic1 = p.Ic1; Ic2 = p.Ic2; 

        tauMag = [p.tau1, p.tau2];
        
        tau = torqueInput(tauMag,t,time_total,type,DOF);

        tau1 = tau(1);
        tau2 = tau(2);
        
        % check that torques are what we wanted
        tau1Array(end+1,:) = [t,tau1];
        tau2Array(end+1,:) = [t,tau2];
        
        %unpacking state
        
        th1 = z(1); th2 = z(2); th1dot = z(3); th2dot = z(4);
        
        %EOM from ddthetaXLag files
        th1dotdot = ddtheta1Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        th2dotdot = ddtheta2Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        
        stateArmDot = [th1dot; th2dot; th1dotdot; th2dotdot];
    end

end