function [r, v, j_theta, j_theta_handEnd, j_theta_handEnd_inv] = deriveKinematicsJacobian_planar_twoLinks()
%the following function derives the kinematics for two-link mechanism in
%the horizontal plane with lengths l1 and l2


syms th1 th2 th1dot th2dot l1 l2 real
% in the above, th are absolute/inertial angles

r1 = l1/2;
r2 = l2/2;

%inertial unit vectors in plane
i = [1 0 ]';
j = [0 1 ]';


rc1 = r1*cos(th1)*i + r1*sin(th1)*j;

re = 2*rc1; %position elbow joint

rc2 = re + r2*cos(th2)*i + r2*sin(th2)*j;

handEnd  = re + l2*cos(th2)*i + l2*sin(th2)*j;

vc1 =  jacobian(rc1,th1)*th1dot ;

vc2 =  jacobian(rc2,th1)*th1dot + jacobian(rc2,th2)*th2dot ;

handEndVel = jacobian(handEnd,th1)*th1dot + jacobian(handEnd,th2)*th2dot ;

r = [rc1; rc2];
v = [vc1; th1dot; vc2; th2dot];

omega = [th1dot; th2dot];

%jacobian wrt omega generalized coordinates, v = J(theta)omega
j_theta = jacobian(v,omega);

%J^-1*v = omega

%handEndVel = J(theta_hand)*omega
j_theta_handEnd = jacobian(handEndVel,omega);

j_theta_handEnd_inv = simplify( pinv(j_theta_handEnd) );



end

