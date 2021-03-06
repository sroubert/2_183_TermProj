close all
clear all
%script for equations of motion
%MAIN DERIVATION


syms th1 th2 th3 th1dot th2dot th3dot th1dotdot th2dotdot th3dotdot real
syms tau1 tau2 tau3 real
syms l1 l2 l3 rfris real
syms thFrisOrient real
syms m1 Ic1 Ic3 m2 Ic2 m3 Ic3 mfris Ifris real

q_vec = [th1, th2, th3];
qdot_vec = [th1dot, th2dot, th3dot];
qdotdot_vec = [th1dotdot th2dotdot th3dotdot];

torque = [tau1, tau2, tau3];

[rCOM, vCOM, j_thetaCOM, j_theta_hand, j_theta_handInv, ...
    handVelocityVec, frisVelocityVec] = ...
    deriveKinematicsJacobian_threeDOF_Fris(th1, th2, th3, th1dot, ...
    th2dot, th3dot, thFrisOrient, l1, l2, l3, rfris, m1, m2, m3, mfris);

%magnitude of position vector of fris COM wrt wrist, law of cosines
rf_wrist = sqrt ( rfris^2 + l3^2 - 2*rfris*l3*cos(pi-thFrisOrient) );

massMat= diag([m1, m1, Ic1, m2, m2, Ic2, m3+mfris, m3+mfris, ...
    Ic3 + Ifris + mfris*rf_wrist^2]);

inertMat = j_thetaCOM'*massMat*j_thetaCOM;

%here lagrangian is only kinetic energy
KE = simplify (0.5*qdot_vec*inertMat*qdot_vec');% = deriveKineticEnergy(qdot_vec, inertMat);
L = KE;

%delL / theta
delL_q = jacobian( L, q_vec')'; %checked

delL_qdot = jacobian(L,qdot_vec)';

d_dt_delL_qdot = jacobian(delL_qdot, [q_vec, qdot_vec])*[qdot_vec, qdotdot_vec]';

E_L_equ = (d_dt_delL_qdot - delL_q - torque' == 0);

equationsThDotDot = solve(E_L_equ, qdotdot_vec); %solve equations of motion

matlabFunction(equationsThDotDot.th1dotdot,'file','ddtheta1_3DOF_fris_Lag'); 
matlabFunction(equationsThDotDot.th2dotdot,'file','ddtheta2_3DOF_fris_Lag'); 
matlabFunction(equationsThDotDot.th3dotdot,'file','ddtheta3_3DOF_fris_Lag'); 

matlabFunction(frisVelocityVec, 'file', 'frisbeeCOMVelocityVector_3DOF');