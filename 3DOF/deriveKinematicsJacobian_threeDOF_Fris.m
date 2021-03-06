function [r, v, j_theta, j_theta_handEnd, j_theta_handEnd_inv, ...
    handEndVel, v_frisCOM_inertial] = ...
    deriveKinematicsJacobian_threeDOF_Fris(th1, th2, th3, th1dot, ...
    th2dot, th3dot, thFrisOrient, l1, l2, l3, rfris, m1, m2, m3, mfris)
%the following function derives the kinematics for three-link mechanism with 
%a frisbee constrained to the mechanism end-point at a given orientaiton,
%thOrient. thOrient is described as a CCW rotation wrt axis along the
%wrist (3rd link)

%all theta are absolute except for thFrisOrient

theta = [th1 th2 th3];
omega = [th1dot; th2dot; th3dot];

r1 = l1/2;
r2 = l2/2;
r3 = l3/2;

%inertial unit vectors in plane
i = [1 0 ]';
j = [0 1 ]';

%unit vectors of frisbee body frame wrt inertial frame
e_rf = [cos(thFrisOrient + th3) sin(thFrisOrient + th3)]';
e_nf = [-sin(thFrisOrient + th3) cos(thFrisOrient + th3)]';

%% position vectors
rc1 = r1*cos(th1)*i + r1*sin(th1)*j;
re = 2*rc1; %position elbow joint
rc2 = re + r2*cos(th2)*i + r2*sin(th2)*j; %2nd link COM
rw = re + l2 * ( cos(th2)*i + sin(th2)*j ); %wrist
rc3 = rw + r3 * ( cos(th3)*i + sin(th3)*j ); %3rd link COM

handEnd  = rw + l3 * ( cos(th3)*i + sin(th3)*j ); %hand end
frisCOM_wrtHandEnd = rfris*e_rf;
frisCOM_inertial = handEnd + frisCOM_wrtHandEnd;

mtot_pastWrist = m3+mfris;

rc3_withFris = (1 / mtot_pastWrist ) * (m3*rc3 + mfris*frisCOM_inertial);

%% velocity vectors
vc1 = jacobian(rc1 ,theta)*omega; 

vc2 =  jacobian(rc2, theta)*omega;

vc3_withFris = jacobian(rc3_withFris,theta)*omega;

handEndVel = jacobian(handEnd,theta)*omega;

%% generalized coordinate vectors

r = [rc1; rc2; rc3_withFris];
v = [vc1; th1dot; vc2; th2dot; vc3_withFris; th3dot];

%% jacobians and velocities of interest
%jacobian wrt omega generalized coordinates, v = J(theta)omega
j_theta = jacobian(v,omega);

j_theta_handEnd = jacobian(handEndVel,omega);

j_theta_handEnd_inv = simplify( pinv(j_theta_handEnd) );

v_frisCOM_inertial = jacobian(frisCOM_inertial ,theta )*omega;

end

