function [tarray, theta1_OL, theta2_OL, theta1dot_OL, theta2dot_OL] = openLoopSim(p, time_total)
%assumes p is a structure containing the parameters of mass, 
%centroidal inertias, lengths, and initial states, ie angles and omegas
%for this also has constant torques


framessec=60; 
tspan=linspace(0,time_total,time_total*framessec);

%unpack initial states from p
th1_0 = p.th1_0; th2_0 = p.th2_0;

th1dot_0 = p.th1dot_0; th2dot_0 = p.th2dot_0;

%initial state array

state0 = [th1_0, th2_0, th1dot_0, th2dot_0];
%integration
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);

[tarray, statearray] = ode45(@RHS, tspan, state0, options, p);

theta1_OL = statearray(:,1); theta2_OL = statearray(:,2);
theta1dot_OL = statearray(:,3); theta2dot_OL = statearray(:,4);


%integration function using Lagrange
    function stateArmDot = RHS(t,z,p) %xdot = Ax + Bu
       %unpacking struct
       l1 = p.l1; l2 = p.l2; m1 = p.m1; m2 = p.m2;

        Ic1 = p.Ic1; Ic2 = p.Ic2; 


        tau1 = p.tau1; tau2 = p.tau2;
        
        %unpacking state
        
        th1 = z(1); th2 = z(2); th1dot = z(3); th2dot = z(4);
        
        %EOM from ddthetaXLag files
        th1dotdot = ddtheta1Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        th2dotdot = ddtheta2Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot);
        
        stateArmDot = [th1dot; th2dot; th1dotdot; th2dotdot];
    end

end