function [r, v, j_theta, j_theta_handEnd, j_theta_handEnd_inv, ...
    handEndVel, v_frisCOM_inertial] = ...
    deriveKinematicsJacobian_planar_twoDOF_Fris(th1, th2, th1dot, th2dot, thfOrient, ...
    l1, l2, rfris, m1, m2, mfris)
%the following function derives the kinematics for two-link mechanism with 
%a frisbee constrained to the mechanism end-point at a given orientaiton,
%thOrient. thOrient is described as a CCW rotation wrt axis along the
%forearm (2nd link)


% syms th1 th2 th1dot th2dot l1 l2 real
% in the above, th are absolute/inertial angles

r1 = l1/2;
r2 = l2/2;

%inertial unit vectors in plane
i = [1 0 ]';
j = [0 1 ]';

%unit vectors of frisbee body frame wrt inertial frame
e_rf = [cos(thfOrient + th2) sin(thfOrient + th2)]';
e_nf = [-sin(thfOrient + th2) cos(thfOrient + th2)]';

%position vectors
rc1 = r1*cos(th1)*i + r1*sin(th1)*j;

re = 2*rc1; %position elbow joint

rc2 = re + r2*cos(th2)*i + r2*sin(th2)*j; %2nd link COM

handEnd  = re + l2*cos(th2)*i + l2*sin(th2)*j;

frisCOM_wrtHandEnd = rfris*e_rf;

frisCOM_inertial = handEnd + frisCOM_wrtHandEnd;

mtot_pastElbow = m2+mfris;

rc2_withFris = (1 / mtot_pastElbow ) * (m2*rc2 + mfris*frisCOM_inertial);

vc1 =  jacobian(rc1,th1)*th1dot ;

vc2_withFris =  jacobian(rc2_withFris,th1)*th1dot + ...
    jacobian(rc2_withFris,th2)*th2dot ;

handEndVel = jacobian(handEnd,th1)*th1dot + jacobian(handEnd,th2)*th2dot ;

r = [rc1; rc2_withFris];
v = [vc1; th1dot; vc2_withFris; th2dot];

omega = [th1dot; th2dot];

%jacobian wrt omega generalized coordinates, v = J(theta)omega
j_theta = jacobian(v,omega);

%J^-1*v = omega

%handEndVel = J(theta_hand)*omega
j_theta_handEnd = jacobian(handEndVel,omega);

j_theta_handEnd_inv = simplify( pinv(j_theta_handEnd) );

v_frisCOM_inertial = jacobian(frisCOM_inertial ,[th1 th2])*omega;

end

