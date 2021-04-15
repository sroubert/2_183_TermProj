close all
clear all
%script for equations of motion

syms th1 th2 th1dot th2dot th1dotdot th2dotdot real
syms tau1 tau2 real
syms l1 l2 real
syms m1 Ic1 m2 Ic2 real

q_vec = [th1, th2];
qdot_vec = [th1dot, th2dot];
qdotdot_vec = [th1dotdot th2dotdot];

torque = [tau1, tau2];

[r_test, v_test, j_theta_test] = deriveKinematicsJacobian_planar_twoLinks();

massMat= deriveMassMatrix_planar_twoLinks();

inertMat = deriveInertiaMat_generalizedCoord(j_theta_test, massMat);

%here lagrangian is only kinetic energy
L = deriveKineticEnergy(qdot_vec, inertMat);

%neville's L 
I_1 = Ic1 + (l1^2*m1)/4;
I_2 = Ic2 + (l2^2*m2)/4;
nev_L = (I_1 + m2*l1^2)*th1dot^2 / 2 + m2*l1*(l2/2)*cos(th2-th1)*th1dot*th2dot + ...
    I_2*th2dot^2 / 2;

%check vs neville's derivation
simplify(L-nev_L)

%delL / theta
delL_q = jacobian( L, q_vec')'; %checked

delL_qdot = jacobian(L,qdot_vec)';

d_dt_delL_qdot = jacobian(delL_qdot, [q_vec, qdot_vec])*[qdot_vec, qdotdot_vec]';

E_L_equ = d_dt_delL_qdot - delL_q - torque';

equationsThDotDot = solve(E_L_equ, qdotdot_vec); %solve equations of motion


matlabFunction(equationsThDotDot.th1dotdot,'file','ddtheta1Lag'); 
matlabFunction(equationsThDotDot.th2dotdot,'file','ddtheta2Lag'); 
