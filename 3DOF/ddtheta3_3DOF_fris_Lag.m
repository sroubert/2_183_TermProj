function out1 = ddtheta3_3DOF_fris_Lag(Ic1,Ic2,Ic3,Ifris,l1,l2,l3,m1,m2,m3,mfris,rfris,tau1,tau2,tau3,th1,th2,th3,th1dot,th2dot,th3dot,thFrisOrient)
%DDTHETA3_3DOF_FRIS_LAG
%    OUT1 = DDTHETA3_3DOF_FRIS_LAG(IC1,IC2,IC3,IFRIS,L1,L2,L3,M1,M2,M3,MFRIS,RFRIS,TAU1,TAU2,TAU3,TH1,TH2,TH3,TH1DOT,TH2DOT,TH3DOT,THFRISORIENT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    10-May-2021 14:00:41

t2 = cos(thFrisOrient);
t3 = l1.^2;
t4 = l1.^3;
t5 = l2.^2;
t6 = l2.^3;
t7 = l3.^2;
t8 = m2.^2;
t9 = m3.^2;
t10 = m3.^3;
t12 = mfris.^2;
t13 = mfris.^3;
t15 = rfris.^2;
t16 = th1dot.^2;
t17 = th2dot.^2;
t18 = th3dot.^2;
t19 = -th1;
t20 = -th2;
t21 = -th3;
t11 = t9.^2;
t14 = t12.^2;
t22 = t20+th1;
t23 = t21+th1;
t24 = t21+th2;
t31 = t19+th3+thFrisOrient;
t32 = t20+th3+thFrisOrient;
t25 = cos(t22);
t26 = cos(t23);
t27 = cos(t24);
t28 = sin(t22);
t29 = sin(t23);
t30 = sin(t24);
t33 = cos(t31);
t34 = cos(t32);
t35 = sin(t31);
t36 = sin(t32);
t37 = t25.^2;
t38 = t26.^2;
t39 = t27.^2;
t40 = t33.^2;
t41 = t34.^2;
out1 = ((m3+mfris).*(Ic1.*Ic2.*tau3.*3.2e+1+t3.*t5.*t8.*tau3.*8.0+t3.*t5.*t9.*tau3.*3.2e+1+t3.*t5.*t12.*tau3.*3.2e+1+Ic2.*m1.*t3.*tau3.*8.0+Ic2.*m2.*t3.*tau3.*3.2e+1+Ic1.*m2.*t5.*tau3.*8.0+Ic2.*m3.*t3.*tau3.*3.2e+1+Ic1.*m3.*t5.*tau3.*3.2e+1+Ic2.*mfris.*t3.*tau3.*3.2e+1+Ic1.*mfris.*t5.*tau3.*3.2e+1+m1.*m2.*t3.*t5.*tau3.*2.0+m1.*m3.*t3.*t5.*tau3.*8.0+m2.*m3.*t3.*t5.*tau3.*4.0e+1+m1.*mfris.*t3.*t5.*tau3.*8.0+m2.*mfris.*t3.*t5.*tau3.*4.0e+1+m3.*mfris.*t3.*t5.*tau3.*6.4e+1-t3.*t5.*t8.*t37.*tau3.*8.0-t3.*t5.*t9.*t37.*tau3.*3.2e+1-t3.*t5.*t12.*t37.*tau3.*3.2e+1-l1.*l3.*t5.*t9.*t26.*tau1.*1.6e+1-l2.*l3.*t3.*t9.*t27.*tau2.*1.6e+1-l1.*l3.*t5.*t12.*t26.*tau1.*3.2e+1-l2.*l3.*t3.*t12.*t27.*tau2.*3.2e+1-m2.*m3.*t3.*t5.*t37.*tau3.*3.2e+1-m2.*mfris.*t3.*t5.*t37.*tau3.*3.2e+1-m3.*mfris.*t3.*t5.*t37.*tau3.*6.4e+1-l1.*rfris.*t5.*t12.*t33.*tau1.*3.2e+1-l2.*rfris.*t3.*t12.*t34.*tau2.*3.2e+1+l3.*t4.*t5.*t10.*t16.*t29.*1.6e+1+l3.*t3.*t6.*t10.*t17.*t30.*1.6e+1+l3.*t4.*t5.*t13.*t16.*t29.*3.2e+1+l3.*t3.*t6.*t13.*t17.*t30.*3.2e+1-rfris.*t4.*t5.*t13.*t16.*t35.*3.2e+1-rfris.*t3.*t6.*t13.*t17.*t36.*3.2e+1-Ic2.*l1.*l3.*m3.*t26.*tau1.*1.6e+1-Ic1.*l2.*l3.*m3.*t27.*tau2.*1.6e+1-Ic2.*l1.*l3.*mfris.*t26.*tau1.*3.2e+1-Ic1.*l2.*l3.*mfris.*t27.*tau2.*3.2e+1-Ic2.*l1.*mfris.*rfris.*t33.*tau1.*3.2e+1-Ic1.*l2.*mfris.*rfris.*t34.*tau2.*3.2e+1+Ic2.*l3.*t4.*t9.*t16.*t29.*1.6e+1+Ic1.*l3.*t6.*t9.*t17.*t30.*1.6e+1+Ic2.*l3.*t4.*t12.*t16.*t29.*3.2e+1+Ic1.*l3.*t6.*t12.*t17.*t30.*3.2e+1-Ic2.*rfris.*t4.*t12.*t16.*t35.*3.2e+1-Ic1.*rfris.*t6.*t12.*t17.*t36.*3.2e+1+Ic1.*Ic2.*l1.*l3.*m3.*t16.*t29.*1.6e+1+Ic1.*Ic2.*l2.*l3.*m3.*t17.*t30.*1.6e+1+Ic1.*Ic2.*l1.*l3.*mfris.*t16.*t29.*3.2e+1+Ic1.*Ic2.*l2.*l3.*mfris.*t17.*t30.*3.2e+1-Ic1.*Ic2.*l1.*mfris.*rfris.*t16.*t35.*3.2e+1-Ic1.*Ic2.*l2.*mfris.*rfris.*t17.*t36.*3.2e+1+Ic2.*l3.*m1.*m3.*t4.*t16.*t29.*4.0+Ic2.*l3.*m2.*m3.*t4.*t16.*t29.*1.6e+1+Ic1.*l3.*m2.*m3.*t6.*t17.*t30.*4.0+Ic2.*l3.*m1.*mfris.*t4.*t16.*t29.*8.0+Ic2.*l3.*m2.*mfris.*t4.*t16.*t29.*3.2e+1+Ic2.*l3.*m3.*mfris.*t4.*t16.*t29.*4.8e+1+Ic1.*l3.*m2.*mfris.*t6.*t17.*t30.*8.0+Ic1.*l3.*m3.*mfris.*t6.*t17.*t30.*4.8e+1+Ic1.*l1.*l3.*t5.*t9.*t16.*t29.*1.6e+1+Ic2.*l2.*l3.*t3.*t9.*t17.*t30.*1.6e+1+Ic1.*l1.*l3.*t5.*t12.*t16.*t29.*3.2e+1+Ic2.*l2.*l3.*t3.*t12.*t17.*t30.*3.2e+1-Ic2.*m1.*mfris.*rfris.*t4.*t16.*t35.*8.0-Ic2.*m2.*mfris.*rfris.*t4.*t16.*t35.*3.2e+1-Ic2.*m3.*mfris.*rfris.*t4.*t16.*t35.*3.2e+1-Ic1.*m2.*mfris.*rfris.*t6.*t17.*t36.*8.0-Ic1.*m3.*mfris.*rfris.*t6.*t17.*t36.*3.2e+1-Ic1.*l1.*rfris.*t5.*t12.*t16.*t35.*3.2e+1-Ic2.*l2.*rfris.*t3.*t12.*t17.*t36.*3.2e+1+Ic2.*t3.*t7.*t9.*t18.*t26.*t29.*8.0+Ic1.*t5.*t7.*t9.*t18.*t27.*t30.*8.0+Ic2.*t3.*t7.*t12.*t18.*t26.*t29.*3.2e+1+Ic1.*t5.*t7.*t12.*t18.*t27.*t30.*3.2e+1-Ic2.*t3.*t12.*t15.*t18.*t33.*t35.*3.2e+1-Ic1.*t5.*t12.*t15.*t18.*t34.*t36.*3.2e+1-l1.*l3.*m2.*m3.*t5.*t26.*tau1.*4.0-l2.*l3.*m1.*m3.*t3.*t27.*tau2.*4.0-l2.*l3.*m2.*m3.*t3.*t27.*tau2.*1.6e+1-l1.*l3.*m2.*mfris.*t5.*t26.*tau1.*8.0-l2.*l3.*m1.*mfris.*t3.*t27.*tau2.*8.0-l1.*l3.*m3.*mfris.*t5.*t26.*tau1.*4.8e+1-l2.*l3.*m2.*mfris.*t3.*t27.*tau2.*3.2e+1-l2.*l3.*m3.*mfris.*t3.*t27.*tau2.*4.8e+1-l1.*m2.*mfris.*rfris.*t5.*t33.*tau1.*8.0-l2.*m1.*mfris.*rfris.*t3.*t34.*tau2.*8.0-l1.*m3.*mfris.*rfris.*t5.*t33.*tau1.*3.2e+1-l2.*m2.*mfris.*rfris.*t3.*t34.*tau2.*3.2e+1-l2.*m3.*mfris.*rfris.*t3.*t34.*tau2.*3.2e+1+l2.*l3.*t3.*t9.*t25.*t26.*tau2.*1.6e+1+l1.*l3.*t5.*t9.*t25.*t27.*tau1.*1.6e+1+l2.*l3.*t3.*t12.*t25.*t26.*tau2.*3.2e+1+l1.*l3.*t5.*t12.*t25.*t27.*tau1.*3.2e+1+l3.*m1.*t4.*t5.*t9.*t16.*t29.*4.0+l3.*m2.*t4.*t5.*t9.*t16.*t29.*2.0e+1+l3.*m3.*t4.*t5.*t8.*t16.*t29.*4.0+l3.*m1.*t3.*t6.*t9.*t17.*t30.*4.0+l3.*m1.*t4.*t5.*t12.*t16.*t29.*8.0+l3.*m2.*t3.*t6.*t9.*t17.*t30.*2.0e+1+l3.*m3.*t3.*t6.*t8.*t17.*t30.*4.0+l3.*m2.*t4.*t5.*t12.*t16.*t29.*4.0e+1+l3.*m1.*t3.*t6.*t12.*t17.*t30.*8.0+l3.*m3.*t4.*t5.*t12.*t16.*t29.*8.0e+1+l3.*m2.*t3.*t6.*t12.*t17.*t30.*4.0e+1+l3.*m3.*t3.*t6.*t12.*t17.*t30.*8.0e+1+l3.*mfris.*t4.*t5.*t8.*t16.*t29.*8.0+l3.*mfris.*t4.*t5.*t9.*t16.*t29.*6.4e+1+l3.*mfris.*t3.*t6.*t8.*t17.*t30.*8.0+l3.*mfris.*t3.*t6.*t9.*t17.*t30.*6.4e+1+l2.*rfris.*t3.*t12.*t25.*t33.*tau2.*3.2e+1+l1.*rfris.*t5.*t12.*t25.*t34.*tau1.*3.2e+1-m1.*rfris.*t4.*t5.*t12.*t16.*t35.*8.0-m2.*rfris.*t4.*t5.*t12.*t16.*t35.*4.0e+1-m1.*rfris.*t3.*t6.*t12.*t17.*t36.*8.0-m3.*rfris.*t4.*t5.*t12.*t16.*t35.*6.4e+1-m2.*rfris.*t3.*t6.*t12.*t17.*t36.*4.0e+1-m3.*rfris.*t3.*t6.*t12.*t17.*t36.*6.4e+1-mfris.*rfris.*t4.*t5.*t8.*t16.*t35.*8.0-mfris.*rfris.*t4.*t5.*t9.*t16.*t35.*3.2e+1-mfris.*rfris.*t3.*t6.*t8.*t17.*t36.*8.0-mfris.*rfris.*t3.*t6.*t9.*t17.*t36.*3.2e+1+l3.*t3.*t6.*t10.*t17.*t26.*t28.*1.6e+1-l3.*t4.*t5.*t10.*t16.*t27.*t28.*1.6e+1+l3.*t3.*t6.*t13.*t17.*t26.*t28.*3.2e+1-l3.*t4.*t5.*t13.*t16.*t27.*t28.*3.2e+1-l3.*t4.*t5.*t10.*t16.*t29.*t37.*1.6e+1-l3.*t3.*t6.*t10.*t17.*t30.*t37.*1.6e+1-l3.*t4.*t5.*t13.*t16.*t29.*t37.*3.2e+1-l3.*t3.*t6.*t13.*t17.*t30.*t37.*3.2e+1+rfris.*t3.*t6.*t13.*t17.*t28.*t33.*3.2e+1-rfris.*t4.*t5.*t13.*t16.*t28.*t34.*3.2e+1+rfris.*t4.*t5.*t13.*t16.*t35.*t37.*3.2e+1+rfris.*t3.*t6.*t13.*t17.*t36.*t37.*3.2e+1+t3.*t5.*t7.*t10.*t18.*t26.*t29.*8.0+t3.*t5.*t7.*t10.*t18.*t27.*t30.*8.0+t3.*t5.*t7.*t13.*t18.*t26.*t29.*3.2e+1+t3.*t5.*t7.*t13.*t18.*t27.*t30.*3.2e+1-t3.*t5.*t13.*t15.*t18.*t33.*t35.*3.2e+1-t3.*t5.*t13.*t15.*t18.*t34.*t36.*3.2e+1+Ic1.*l1.*l3.*m2.*m3.*t5.*t16.*t29.*4.0+Ic2.*l2.*l3.*m1.*m3.*t3.*t17.*t30.*4.0+Ic2.*l2.*l3.*m2.*m3.*t3.*t17.*t30.*1.6e+1+Ic1.*l1.*l3.*m2.*mfris.*t5.*t16.*t29.*8.0+Ic1.*l1.*l3.*m3.*mfris.*t5.*t16.*t29.*4.8e+1+Ic2.*l2.*l3.*m1.*mfris.*t3.*t17.*t30.*8.0+Ic2.*l2.*l3.*m2.*mfris.*t3.*t17.*t30.*3.2e+1+Ic2.*l2.*l3.*m3.*mfris.*t3.*t17.*t30.*4.8e+1-Ic1.*l1.*m2.*mfris.*rfris.*t5.*t16.*t35.*8.0-Ic1.*l1.*m3.*mfris.*rfris.*t5.*t16.*t35.*3.2e+1-Ic2.*l2.*m1.*mfris.*rfris.*t3.*t17.*t36.*8.0-Ic2.*l2.*m2.*mfris.*rfris.*t3.*t17.*t36.*3.2e+1-Ic2.*l2.*m3.*mfris.*rfris.*t3.*t17.*t36.*3.2e+1-Ic1.*l1.*l3.*t5.*t9.*t16.*t27.*t28.*1.6e+1+Ic2.*l2.*l3.*t3.*t9.*t17.*t26.*t28.*1.6e+1-Ic1.*l1.*l3.*t5.*t12.*t16.*t27.*t28.*3.2e+1+Ic2.*l2.*l3.*t3.*t12.*t17.*t26.*t28.*3.2e+1+Ic2.*m3.*mfris.*t3.*t7.*t18.*t26.*t29.*3.2e+1+Ic1.*m3.*mfris.*t5.*t7.*t18.*t27.*t30.*3.2e+1-Ic1.*l1.*rfris.*t5.*t12.*t16.*t28.*t34.*3.2e+1+Ic2.*l2.*rfris.*t3.*t12.*t17.*t28.*t33.*3.2e+1-Ic2.*l3.*rfris.*t3.*t12.*t18.*t26.*t35.*3.2e+1+Ic2.*l3.*rfris.*t3.*t12.*t18.*t29.*t33.*3.2e+1-Ic1.*l3.*rfris.*t5.*t12.*t18.*t27.*t36.*3.2e+1+Ic1.*l3.*rfris.*t5.*t12.*t18.*t30.*t34.*3.2e+1+l2.*l3.*m2.*m3.*t3.*t25.*t26.*tau2.*8.0+l1.*l3.*m2.*m3.*t5.*t25.*t27.*tau1.*8.0+l2.*l3.*m2.*mfris.*t3.*t25.*t26.*tau2.*1.6e+1+l1.*l3.*m2.*mfris.*t5.*t25.*t27.*tau1.*1.6e+1+l2.*l3.*m3.*mfris.*t3.*t25.*t26.*tau2.*4.8e+1+l1.*l3.*m3.*mfris.*t5.*t25.*t27.*tau1.*4.8e+1+l3.*m1.*m2.*m3.*t4.*t5.*t16.*t29+l3.*m1.*m2.*m3.*t3.*t6.*t17.*t30+l3.*m1.*m2.*mfris.*t4.*t5.*t16.*t29.*2.0+l3.*m1.*m3.*mfris.*t4.*t5.*t16.*t29.*1.2e+1+l3.*m1.*m2.*mfris.*t3.*t6.*t17.*t30.*2.0+l3.*m2.*m3.*mfris.*t4.*t5.*t16.*t29.*6.0e+1+l3.*m1.*m3.*mfris.*t3.*t6.*t17.*t30.*1.2e+1+l3.*m2.*m3.*mfris.*t3.*t6.*t17.*t30.*6.0e+1+l2.*m2.*mfris.*rfris.*t3.*t25.*t33.*tau2.*1.6e+1+l1.*m2.*mfris.*rfris.*t5.*t25.*t34.*tau1.*1.6e+1+l2.*m3.*mfris.*rfris.*t3.*t25.*t33.*tau2.*3.2e+1+l1.*m3.*mfris.*rfris.*t5.*t25.*t34.*tau1.*3.2e+1-m1.*m2.*mfris.*rfris.*t4.*t5.*t16.*t35.*2.0-m1.*m3.*mfris.*rfris.*t4.*t5.*t16.*t35.*8.0-m1.*m2.*mfris.*rfris.*t3.*t6.*t17.*t36.*2.0-m2.*m3.*mfris.*rfris.*t4.*t5.*t16.*t35.*4.0e+1-m1.*m3.*mfris.*rfris.*t3.*t6.*t17.*t36.*8.0-m2.*m3.*mfris.*rfris.*t3.*t6.*t17.*t36.*4.0e+1-l3.*m1.*t4.*t5.*t9.*t16.*t27.*t28.*4.0+l3.*m2.*t3.*t6.*t9.*t17.*t26.*t28.*1.2e+1-l3.*m2.*t4.*t5.*t9.*t16.*t27.*t28.*2.4e+1+l3.*m3.*t3.*t6.*t8.*t17.*t26.*t28.*2.0-l3.*m3.*t4.*t5.*t8.*t16.*t27.*t28.*8.0-l3.*m1.*t4.*t5.*t12.*t16.*t27.*t28.*8.0+l3.*m2.*t3.*t6.*t12.*t17.*t26.*t28.*2.4e+1-l3.*m2.*t4.*t5.*t12.*t16.*t27.*t28.*4.8e+1+l3.*m3.*t3.*t6.*t12.*t17.*t26.*t28.*8.0e+1-l3.*m3.*t4.*t5.*t12.*t16.*t27.*t28.*8.0e+1-l3.*m2.*t4.*t5.*t9.*t16.*t29.*t37.*1.6e+1-l3.*m3.*t4.*t5.*t8.*t16.*t29.*t37.*4.0-l3.*m2.*t3.*t6.*t9.*t17.*t30.*t37.*1.6e+1-l3.*m3.*t3.*t6.*t8.*t17.*t30.*t37.*4.0-l3.*m2.*t4.*t5.*t12.*t16.*t29.*t37.*3.2e+1-l3.*m3.*t4.*t5.*t12.*t16.*t29.*t37.*8.0e+1-l3.*m2.*t3.*t6.*t12.*t17.*t30.*t37.*3.2e+1-l3.*m3.*t3.*t6.*t12.*t17.*t30.*t37.*8.0e+1+l3.*mfris.*t3.*t6.*t8.*t17.*t26.*t28.*4.0-l3.*mfris.*t4.*t5.*t8.*t16.*t27.*t28.*1.6e+1+l3.*mfris.*t3.*t6.*t9.*t17.*t26.*t28.*6.4e+1-l3.*mfris.*t4.*t5.*t9.*t16.*t27.*t28.*6.4e+1-l3.*mfris.*t4.*t5.*t8.*t16.*t29.*t37.*8.0-l3.*mfris.*t4.*t5.*t9.*t16.*t29.*t37.*6.4e+1-l3.*mfris.*t3.*t6.*t8.*t17.*t30.*t37.*8.0-l3.*mfris.*t3.*t6.*t9.*t17.*t30.*t37.*6.4e+1-l3.*rfris.*t3.*t5.*t13.*t18.*t26.*t35.*3.2e+1+l3.*rfris.*t3.*t5.*t13.*t18.*t29.*t33.*3.2e+1-l3.*rfris.*t3.*t5.*t13.*t18.*t27.*t36.*3.2e+1+l3.*rfris.*t3.*t5.*t13.*t18.*t30.*t34.*3.2e+1-m1.*rfris.*t4.*t5.*t12.*t16.*t28.*t34.*8.0+m2.*rfris.*t3.*t6.*t12.*t17.*t28.*t33.*2.4e+1-m2.*rfris.*t4.*t5.*t12.*t16.*t28.*t34.*4.8e+1+m3.*rfris.*t3.*t6.*t12.*t17.*t28.*t33.*6.4e+1-m3.*rfris.*t4.*t5.*t12.*t16.*t28.*t34.*6.4e+1+m2.*rfris.*t4.*t5.*t12.*t16.*t35.*t37.*3.2e+1+m3.*rfris.*t4.*t5.*t12.*t16.*t35.*t37.*6.4e+1+m2.*rfris.*t3.*t6.*t12.*t17.*t36.*t37.*3.2e+1+m3.*rfris.*t3.*t6.*t12.*t17.*t36.*t37.*6.4e+1+mfris.*rfris.*t3.*t6.*t8.*t17.*t28.*t33.*4.0-mfris.*rfris.*t4.*t5.*t8.*t16.*t28.*t34.*1.6e+1+mfris.*rfris.*t3.*t6.*t9.*t17.*t28.*t33.*3.2e+1-mfris.*rfris.*t4.*t5.*t9.*t16.*t28.*t34.*3.2e+1+mfris.*rfris.*t4.*t5.*t8.*t16.*t35.*t37.*8.0+mfris.*rfris.*t4.*t5.*t9.*t16.*t35.*t37.*3.2e+1+mfris.*rfris.*t3.*t6.*t8.*t17.*t36.*t37.*8.0+mfris.*rfris.*t3.*t6.*t9.*t17.*t36.*t37.*3.2e+1+l3.*t4.*t5.*t10.*t16.*t25.*t26.*t28.*1.6e+1-l3.*t3.*t6.*t10.*t17.*t25.*t27.*t28.*1.6e+1+l3.*t4.*t5.*t13.*t16.*t25.*t26.*t28.*3.2e+1-l3.*t3.*t6.*t13.*t17.*t25.*t27.*t28.*3.2e+1+m2.*t3.*t5.*t7.*t9.*t18.*t26.*t29.*2.0+m1.*t3.*t5.*t7.*t9.*t18.*t27.*t30.*2.0+m2.*t3.*t5.*t7.*t9.*t18.*t27.*t30.*8.0+m2.*t3.*t5.*t7.*t12.*t18.*t26.*t29.*8.0+m1.*t3.*t5.*t7.*t12.*t18.*t27.*t30.*8.0+m3.*t3.*t5.*t7.*t12.*t18.*t26.*t29.*6.4e+1+m2.*t3.*t5.*t7.*t12.*t18.*t27.*t30.*3.2e+1+m3.*t3.*t5.*t7.*t12.*t18.*t27.*t30.*6.4e+1-m2.*t3.*t5.*t12.*t15.*t18.*t33.*t35.*8.0-m1.*t3.*t5.*t12.*t15.*t18.*t34.*t36.*8.0-m3.*t3.*t5.*t12.*t15.*t18.*t33.*t35.*3.2e+1-m2.*t3.*t5.*t12.*t15.*t18.*t34.*t36.*3.2e+1-m3.*t3.*t5.*t12.*t15.*t18.*t34.*t36.*3.2e+1+mfris.*t3.*t5.*t7.*t9.*t18.*t26.*t29.*4.0e+1+mfris.*t3.*t5.*t7.*t9.*t18.*t27.*t30.*4.0e+1+rfris.*t4.*t5.*t13.*t16.*t25.*t28.*t33.*3.2e+1-rfris.*t3.*t6.*t13.*t17.*t25.*t28.*t34.*3.2e+1-t3.*t5.*t7.*t10.*t18.*t25.*t26.*t30.*8.0-t3.*t5.*t7.*t10.*t18.*t25.*t27.*t29.*8.0-t3.*t5.*t7.*t13.*t18.*t25.*t26.*t30.*3.2e+1-t3.*t5.*t7.*t13.*t18.*t25.*t27.*t29.*3.2e+1+t3.*t5.*t13.*t15.*t18.*t25.*t33.*t36.*3.2e+1+t3.*t5.*t13.*t15.*t18.*t25.*t34.*t35.*3.2e+1-Ic1.*l1.*l3.*m2.*m3.*t5.*t16.*t27.*t28.*8.0+Ic2.*l2.*l3.*m2.*m3.*t3.*t17.*t26.*t28.*8.0-Ic1.*l1.*l3.*m2.*mfris.*t5.*t16.*t27.*t28.*1.6e+1+Ic2.*l2.*l3.*m2.*mfris.*t3.*t17.*t26.*t28.*1.6e+1-Ic1.*l1.*l3.*m3.*mfris.*t5.*t16.*t27.*t28.*4.8e+1+Ic2.*l2.*l3.*m3.*mfris.*t3.*t17.*t26.*t28.*4.8e+1-Ic1.*l1.*m2.*mfris.*rfris.*t5.*t16.*t28.*t34.*1.6e+1+Ic2.*l2.*m2.*mfris.*rfris.*t3.*t17.*t28.*t33.*1.6e+1-Ic1.*l1.*m3.*mfris.*rfris.*t5.*t16.*t28.*t34.*3.2e+1+Ic2.*l2.*m3.*mfris.*rfris.*t3.*t17.*t28.*t33.*3.2e+1-Ic2.*l3.*m3.*mfris.*rfris.*t3.*t18.*t26.*t35.*1.6e+1+Ic2.*l3.*m3.*mfris.*rfris.*t3.*t18.*t29.*t33.*1.6e+1-Ic1.*l3.*m3.*mfris.*rfris.*t5.*t18.*t27.*t36.*1.6e+1+Ic1.*l3.*m3.*mfris.*rfris.*t5.*t18.*t30.*t34.*1.6e+1-l3.*m1.*m2.*m3.*t4.*t5.*t16.*t27.*t28.*2.0-l3.*m1.*m2.*mfris.*t4.*t5.*t16.*t27.*t28.*4.0-l3.*m1.*m3.*mfris.*t4.*t5.*t16.*t27.*t28.*1.2e+1+l3.*m2.*m3.*mfris.*t3.*t6.*t17.*t26.*t28.*3.6e+1-l3.*m2.*m3.*mfris.*t4.*t5.*t16.*t27.*t28.*7.2e+1-l3.*m2.*m3.*mfris.*t4.*t5.*t16.*t29.*t37.*4.8e+1-l3.*m2.*m3.*mfris.*t3.*t6.*t17.*t30.*t37.*4.8e+1-m1.*m2.*mfris.*rfris.*t4.*t5.*t16.*t28.*t34.*4.0-m1.*m3.*mfris.*rfris.*t4.*t5.*t16.*t28.*t34.*8.0+m2.*m3.*mfris.*rfris.*t3.*t6.*t17.*t28.*t33.*2.4e+1-m2.*m3.*mfris.*rfris.*t4.*t5.*t16.*t28.*t34.*4.8e+1+m2.*m3.*mfris.*rfris.*t4.*t5.*t16.*t35.*t37.*3.2e+1+m2.*m3.*mfris.*rfris.*t3.*t6.*t17.*t36.*t37.*3.2e+1+m2.*m3.*mfris.*t3.*t5.*t7.*t18.*t26.*t29.*8.0+m1.*m3.*mfris.*t3.*t5.*t7.*t18.*t27.*t30.*8.0+m2.*m3.*mfris.*t3.*t5.*t7.*t18.*t27.*t30.*3.2e+1-l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t26.*t35.*8.0-l3.*m1.*rfris.*t3.*t5.*t12.*t18.*t27.*t36.*8.0+l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t29.*t33.*8.0-l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t26.*t35.*4.8e+1+l3.*m1.*rfris.*t3.*t5.*t12.*t18.*t30.*t34.*8.0-l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t27.*t36.*3.2e+1+l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t29.*t33.*4.8e+1+l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t30.*t34.*3.2e+1-l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t27.*t36.*4.8e+1+l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t30.*t34.*4.8e+1-l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t26.*t35.*1.6e+1+l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t29.*t33.*1.6e+1-l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t27.*t36.*1.6e+1+l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t30.*t34.*1.6e+1+l3.*m2.*t4.*t5.*t9.*t16.*t25.*t26.*t28.*1.6e+1+l3.*m3.*t4.*t5.*t8.*t16.*t25.*t26.*t28.*4.0-l3.*m2.*t3.*t6.*t9.*t17.*t25.*t27.*t28.*1.6e+1-l3.*m3.*t3.*t6.*t8.*t17.*t25.*t27.*t28.*4.0+l3.*m2.*t4.*t5.*t12.*t16.*t25.*t26.*t28.*3.2e+1+l3.*m3.*t4.*t5.*t12.*t16.*t25.*t26.*t28.*8.0e+1-l3.*m2.*t3.*t6.*t12.*t17.*t25.*t27.*t28.*3.2e+1-l3.*m3.*t3.*t6.*t12.*t17.*t25.*t27.*t28.*8.0e+1+l3.*mfris.*t4.*t5.*t8.*t16.*t25.*t26.*t28.*8.0+l3.*mfris.*t4.*t5.*t9.*t16.*t25.*t26.*t28.*6.4e+1-l3.*mfris.*t3.*t6.*t8.*t17.*t25.*t27.*t28.*8.0-l3.*mfris.*t3.*t6.*t9.*t17.*t25.*t27.*t28.*6.4e+1+l3.*rfris.*t3.*t5.*t13.*t18.*t25.*t26.*t36.*3.2e+1+l3.*rfris.*t3.*t5.*t13.*t18.*t25.*t27.*t35.*3.2e+1-l3.*rfris.*t3.*t5.*t13.*t18.*t25.*t29.*t34.*3.2e+1-l3.*rfris.*t3.*t5.*t13.*t18.*t25.*t30.*t33.*3.2e+1+m2.*rfris.*t4.*t5.*t12.*t16.*t25.*t28.*t33.*3.2e+1+m3.*rfris.*t4.*t5.*t12.*t16.*t25.*t28.*t33.*6.4e+1-m2.*rfris.*t3.*t6.*t12.*t17.*t25.*t28.*t34.*3.2e+1-m3.*rfris.*t3.*t6.*t12.*t17.*t25.*t28.*t34.*6.4e+1+mfris.*rfris.*t4.*t5.*t8.*t16.*t25.*t28.*t33.*8.0+mfris.*rfris.*t4.*t5.*t9.*t16.*t25.*t28.*t33.*3.2e+1-mfris.*rfris.*t3.*t6.*t8.*t17.*t25.*t28.*t34.*8.0-mfris.*rfris.*t3.*t6.*t9.*t17.*t25.*t28.*t34.*3.2e+1-m2.*t3.*t5.*t7.*t9.*t18.*t25.*t26.*t30.*4.0-m2.*t3.*t5.*t7.*t9.*t18.*t25.*t27.*t29.*4.0-m2.*t3.*t5.*t7.*t12.*t18.*t25.*t26.*t30.*1.6e+1-m2.*t3.*t5.*t7.*t12.*t18.*t25.*t27.*t29.*1.6e+1-m3.*t3.*t5.*t7.*t12.*t18.*t25.*t26.*t30.*6.4e+1-m3.*t3.*t5.*t7.*t12.*t18.*t25.*t27.*t29.*6.4e+1+m2.*t3.*t5.*t12.*t15.*t18.*t25.*t33.*t36.*1.6e+1+m2.*t3.*t5.*t12.*t15.*t18.*t25.*t34.*t35.*1.6e+1+m3.*t3.*t5.*t12.*t15.*t18.*t25.*t33.*t36.*3.2e+1+m3.*t3.*t5.*t12.*t15.*t18.*t25.*t34.*t35.*3.2e+1-mfris.*t3.*t5.*t7.*t9.*t18.*t25.*t26.*t30.*4.0e+1-mfris.*t3.*t5.*t7.*t9.*t18.*t25.*t27.*t29.*4.0e+1-l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t26.*t35.*4.0-l3.*m1.*m3.*mfris.*rfris.*t3.*t5.*t18.*t27.*t36.*4.0+l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t29.*t33.*4.0+l3.*m1.*m3.*mfris.*rfris.*t3.*t5.*t18.*t30.*t34.*4.0-l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t27.*t36.*1.6e+1+l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t30.*t34.*1.6e+1+l3.*m2.*m3.*mfris.*t4.*t5.*t16.*t25.*t26.*t28.*4.8e+1-l3.*m2.*m3.*mfris.*t3.*t6.*t17.*t25.*t27.*t28.*4.8e+1+m2.*m3.*mfris.*rfris.*t4.*t5.*t16.*t25.*t28.*t33.*3.2e+1-m2.*m3.*mfris.*rfris.*t3.*t6.*t17.*t25.*t28.*t34.*3.2e+1-m2.*m3.*mfris.*t3.*t5.*t7.*t18.*t25.*t26.*t30.*1.6e+1-m2.*m3.*mfris.*t3.*t5.*t7.*t18.*t25.*t27.*t29.*1.6e+1+l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t25.*t26.*t36.*1.6e+1+l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t25.*t27.*t35.*1.6e+1-l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t25.*t29.*t34.*1.6e+1-l3.*m2.*rfris.*t3.*t5.*t12.*t18.*t25.*t30.*t33.*1.6e+1+l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t25.*t26.*t36.*4.8e+1+l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t25.*t27.*t35.*4.8e+1-l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t25.*t29.*t34.*4.8e+1-l3.*m3.*rfris.*t3.*t5.*t12.*t18.*t25.*t30.*t33.*4.8e+1+l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t25.*t26.*t36.*1.6e+1+l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t25.*t27.*t35.*1.6e+1-l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t25.*t29.*t34.*1.6e+1-l3.*mfris.*rfris.*t3.*t5.*t9.*t18.*t25.*t30.*t33.*1.6e+1+l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t25.*t26.*t36.*8.0+l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t25.*t27.*t35.*8.0-l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t25.*t29.*t34.*8.0-l3.*m2.*m3.*mfris.*rfris.*t3.*t5.*t18.*t25.*t30.*t33.*8.0).*2.0)./(Ic3.*t3.*t5.*t10.*6.4e+1+Ic2.*t3.*t7.*t10.*1.6e+1+Ic1.*t5.*t7.*t10.*1.6e+1+Ic3.*t3.*t5.*t13.*6.4e+1+Ic2.*t3.*t7.*t13.*1.28e+2+Ic1.*t5.*t7.*t13.*1.28e+2+Ic2.*t3.*t13.*t15.*1.28e+2+Ic1.*t5.*t13.*t15.*1.28e+2+Ifris.*t3.*t5.*t10.*6.4e+1+Ifris.*t3.*t5.*t13.*6.4e+1+t3.*t5.*t7.*t11.*1.6e+1+t3.*t5.*t7.*t14.*1.28e+2+t3.*t5.*t14.*t15.*1.28e+2+Ic1.*Ic2.*Ic3.*m3.*6.4e+1+Ic1.*Ic2.*Ifris.*m3.*6.4e+1+Ic1.*Ic2.*Ic3.*mfris.*6.4e+1+Ic1.*Ic2.*Ifris.*mfris.*6.4e+1+Ic2.*Ic3.*t3.*t9.*6.4e+1+Ic1.*Ic3.*t5.*t9.*6.4e+1+Ic1.*Ic2.*t7.*t9.*1.6e+1+Ic2.*Ic3.*t3.*t12.*6.4e+1+Ic1.*Ic3.*t5.*t12.*6.4e+1+Ic1.*Ic2.*t7.*t12.*1.28e+2+Ic1.*Ic2.*t12.*t15.*1.28e+2+Ic2.*Ifris.*t3.*t9.*6.4e+1+Ic1.*Ifris.*t5.*t9.*6.4e+1+Ic2.*Ifris.*t3.*t12.*6.4e+1+Ic1.*Ifris.*t5.*t12.*6.4e+1+Ic2.*Ic3.*m1.*m3.*t3.*1.6e+1+Ic2.*Ic3.*m2.*m3.*t3.*6.4e+1+Ic1.*Ic3.*m2.*m3.*t5.*1.6e+1+Ic2.*Ifris.*m1.*m3.*t3.*1.6e+1+Ic2.*Ifris.*m2.*m3.*t3.*6.4e+1+Ic1.*Ifris.*m2.*m3.*t5.*1.6e+1+Ic2.*Ic3.*m1.*mfris.*t3.*1.6e+1+Ic2.*Ic3.*m2.*mfris.*t3.*6.4e+1+Ic1.*Ic3.*m2.*mfris.*t5.*1.6e+1+Ic2.*Ic3.*m3.*mfris.*t3.*1.28e+2+Ic1.*Ic3.*m3.*mfris.*t5.*1.28e+2+Ic1.*Ic2.*m3.*mfris.*t7.*1.28e+2+Ic1.*Ic2.*m3.*mfris.*t15.*6.4e+1+Ic2.*Ifris.*m1.*mfris.*t3.*1.6e+1+Ic2.*Ifris.*m2.*mfris.*t3.*6.4e+1+Ic1.*Ifris.*m2.*mfris.*t5.*1.6e+1+Ic2.*Ifris.*m3.*mfris.*t3.*1.28e+2+Ic1.*Ifris.*m3.*mfris.*t5.*1.28e+2+Ic3.*m1.*t3.*t5.*t9.*1.6e+1+Ic2.*m1.*t3.*t7.*t9.*4.0+Ic3.*m2.*t3.*t5.*t9.*8.0e+1+Ic3.*m3.*t3.*t5.*t8.*1.6e+1+Ic2.*m2.*t3.*t7.*t9.*1.6e+1+Ic1.*m2.*t5.*t7.*t9.*4.0+Ic3.*m1.*t3.*t5.*t12.*1.6e+1+Ic2.*m1.*t3.*t7.*t12.*3.2e+1+Ic3.*m2.*t3.*t5.*t12.*8.0e+1+Ic2.*m2.*t3.*t7.*t12.*1.28e+2+Ic3.*m3.*t3.*t5.*t12.*1.92e+2+Ic1.*m2.*t5.*t7.*t12.*3.2e+1+Ic2.*m3.*t3.*t7.*t12.*2.56e+2+Ic1.*m3.*t5.*t7.*t12.*2.56e+2+Ic2.*m1.*t3.*t12.*t15.*3.2e+1+Ic2.*m2.*t3.*t12.*t15.*1.28e+2+Ic1.*m2.*t5.*t12.*t15.*3.2e+1+Ic2.*m3.*t3.*t12.*t15.*1.92e+2+Ic1.*m3.*t5.*t12.*t15.*1.92e+2+Ifris.*m1.*t3.*t5.*t9.*1.6e+1+Ifris.*m2.*t3.*t5.*t9.*8.0e+1+Ifris.*m3.*t3.*t5.*t8.*1.6e+1+Ifris.*m1.*t3.*t5.*t12.*1.6e+1+Ifris.*m2.*t3.*t5.*t12.*8.0e+1+Ifris.*m3.*t3.*t5.*t12.*1.92e+2+Ic3.*mfris.*t3.*t5.*t8.*1.6e+1+Ic3.*mfris.*t3.*t5.*t9.*1.92e+2+Ic2.*mfris.*t3.*t7.*t9.*1.44e+2+Ic1.*mfris.*t5.*t7.*t9.*1.44e+2+Ic2.*mfris.*t3.*t9.*t15.*6.4e+1+Ic1.*mfris.*t5.*t9.*t15.*6.4e+1+Ifris.*mfris.*t3.*t5.*t8.*1.6e+1+Ifris.*mfris.*t3.*t5.*t9.*1.92e+2-Ic3.*t3.*t5.*t10.*t37.*6.4e+1-Ic2.*t3.*t7.*t10.*t38.*1.6e+1-Ic3.*t3.*t5.*t13.*t37.*6.4e+1-Ic1.*t5.*t7.*t10.*t39.*1.6e+1-Ic2.*t3.*t7.*t13.*t38.*6.4e+1-Ic1.*t5.*t7.*t13.*t39.*6.4e+1-Ic2.*t3.*t13.*t15.*t40.*6.4e+1-Ic1.*t5.*t13.*t15.*t41.*6.4e+1-Ifris.*t3.*t5.*t10.*t37.*6.4e+1-Ifris.*t3.*t5.*t13.*t37.*6.4e+1+m1.*t3.*t5.*t7.*t10.*4.0+m2.*t3.*t5.*t7.*t10.*2.0e+1+m1.*t3.*t5.*t7.*t13.*3.2e+1+m2.*t3.*t5.*t7.*t13.*1.6e+2+m3.*t3.*t5.*t7.*t13.*3.84e+2+m1.*t3.*t5.*t13.*t15.*3.2e+1+m2.*t3.*t5.*t13.*t15.*1.6e+2+m3.*t3.*t5.*t13.*t15.*3.2e+2+mfris.*t3.*t5.*t7.*t10.*1.6e+2+mfris.*t3.*t5.*t10.*t15.*6.4e+1+t3.*t5.*t7.*t8.*t9.*4.0+t3.*t5.*t7.*t8.*t12.*3.2e+1+t3.*t5.*t7.*t9.*t12.*4.0e+2+t3.*t5.*t8.*t12.*t15.*3.2e+1+t3.*t5.*t9.*t12.*t15.*2.56e+2-t3.*t5.*t7.*t11.*t37.*1.6e+1-t3.*t5.*t7.*t11.*t38.*1.6e+1-t3.*t5.*t7.*t11.*t39.*1.6e+1-t3.*t5.*t7.*t14.*t37.*1.28e+2-t3.*t5.*t7.*t14.*t38.*6.4e+1-t3.*t5.*t7.*t14.*t39.*6.4e+1-t3.*t5.*t14.*t15.*t37.*1.28e+2-t3.*t5.*t14.*t15.*t40.*6.4e+1-t3.*t5.*t14.*t15.*t41.*6.4e+1+m1.*m2.*t3.*t5.*t7.*t9+m1.*m2.*t3.*t5.*t7.*t12.*8.0+m1.*m3.*t3.*t5.*t7.*t12.*6.4e+1+m2.*m3.*t3.*t5.*t7.*t12.*3.2e+2+m1.*m2.*t3.*t5.*t12.*t15.*8.0+m1.*m3.*t3.*t5.*t12.*t15.*4.8e+1+m2.*m3.*t3.*t5.*t12.*t15.*2.4e+2+m1.*mfris.*t3.*t5.*t7.*t9.*3.6e+1+m2.*mfris.*t3.*t5.*t7.*t9.*1.8e+2+m3.*mfris.*t3.*t5.*t7.*t8.*3.2e+1+m1.*mfris.*t3.*t5.*t9.*t15.*1.6e+1+m2.*mfris.*t3.*t5.*t9.*t15.*8.0e+1+m3.*mfris.*t3.*t5.*t8.*t15.*1.6e+1+l3.*rfris.*t2.*t3.*t5.*t14.*2.56e+2-m2.*t3.*t5.*t7.*t10.*t37.*1.6e+1-m1.*t3.*t5.*t7.*t10.*t39.*4.0-m2.*t3.*t5.*t7.*t10.*t38.*4.0-m2.*t3.*t5.*t7.*t10.*t39.*1.6e+1-m2.*t3.*t5.*t7.*t13.*t37.*1.28e+2-m1.*t3.*t5.*t7.*t13.*t39.*1.6e+1-m2.*t3.*t5.*t7.*t13.*t38.*1.6e+1-m3.*t3.*t5.*t7.*t13.*t37.*3.84e+2-m2.*t3.*t5.*t7.*t13.*t39.*6.4e+1-m3.*t3.*t5.*t7.*t13.*t38.*1.92e+2-m3.*t3.*t5.*t7.*t13.*t39.*1.92e+2-m2.*t3.*t5.*t13.*t15.*t37.*1.28e+2-m3.*t3.*t5.*t13.*t15.*t37.*3.2e+2-m1.*t3.*t5.*t13.*t15.*t41.*1.6e+1-m2.*t3.*t5.*t13.*t15.*t40.*1.6e+1-m2.*t3.*t5.*t13.*t15.*t41.*6.4e+1-m3.*t3.*t5.*t13.*t15.*t40.*1.28e+2-m3.*t3.*t5.*t13.*t15.*t41.*1.28e+2-mfris.*t3.*t5.*t7.*t10.*t37.*1.6e+2-mfris.*t3.*t5.*t7.*t10.*t38.*9.6e+1-mfris.*t3.*t5.*t7.*t10.*t39.*9.6e+1-mfris.*t3.*t5.*t10.*t15.*t37.*6.4e+1-t3.*t5.*t7.*t8.*t9.*t37.*4.0-t3.*t5.*t7.*t8.*t12.*t37.*3.2e+1-t3.*t5.*t7.*t9.*t12.*t37.*4.0e+2-t3.*t5.*t7.*t9.*t12.*t38.*2.08e+2-t3.*t5.*t7.*t9.*t12.*t39.*2.08e+2-t3.*t5.*t8.*t12.*t15.*t37.*3.2e+1-t3.*t5.*t9.*t12.*t15.*t37.*2.56e+2-t3.*t5.*t9.*t12.*t15.*t40.*6.4e+1-t3.*t5.*t9.*t12.*t15.*t41.*6.4e+1+Ic1.*Ic2.*l3.*rfris.*t2.*t12.*2.56e+2+Ic3.*m1.*m2.*m3.*t3.*t5.*4.0+Ifris.*m1.*m2.*m3.*t3.*t5.*4.0+Ic3.*m1.*m2.*mfris.*t3.*t5.*4.0+Ic3.*m1.*m3.*mfris.*t3.*t5.*3.2e+1+Ic2.*m1.*m3.*mfris.*t3.*t7.*3.2e+1+Ic3.*m2.*m3.*mfris.*t3.*t5.*1.6e+2+Ic2.*m2.*m3.*mfris.*t3.*t7.*1.28e+2+Ic1.*m2.*m3.*mfris.*t5.*t7.*3.2e+1+Ic2.*m1.*m3.*mfris.*t3.*t15.*1.6e+1+Ic2.*m2.*m3.*mfris.*t3.*t15.*6.4e+1+Ic1.*m2.*m3.*mfris.*t5.*t15.*1.6e+1+Ifris.*m1.*m2.*mfris.*t3.*t5.*4.0+Ifris.*m1.*m3.*mfris.*t3.*t5.*3.2e+1+Ifris.*m2.*m3.*mfris.*t3.*t5.*1.6e+2+Ic2.*l3.*rfris.*t2.*t3.*t13.*2.56e+2+Ic1.*l3.*rfris.*t2.*t5.*t13.*2.56e+2-Ic3.*m2.*t3.*t5.*t9.*t37.*6.4e+1-Ic3.*m3.*t3.*t5.*t8.*t37.*1.6e+1-Ic3.*m2.*t3.*t5.*t12.*t37.*6.4e+1-Ic3.*m3.*t3.*t5.*t12.*t37.*1.92e+2-Ic2.*m3.*t3.*t7.*t12.*t38.*1.28e+2-Ic1.*m3.*t5.*t7.*t12.*t39.*1.28e+2-Ic2.*m3.*t3.*t12.*t15.*t40.*6.4e+1-Ic1.*m3.*t5.*t12.*t15.*t41.*6.4e+1-Ifris.*m2.*t3.*t5.*t9.*t37.*6.4e+1-Ifris.*m3.*t3.*t5.*t8.*t37.*1.6e+1-Ifris.*m2.*t3.*t5.*t12.*t37.*6.4e+1-Ifris.*m3.*t3.*t5.*t12.*t37.*1.92e+2-Ic3.*mfris.*t3.*t5.*t8.*t37.*1.6e+1-Ic3.*mfris.*t3.*t5.*t9.*t37.*1.92e+2-Ic2.*mfris.*t3.*t7.*t9.*t38.*8.0e+1-Ic1.*mfris.*t5.*t7.*t9.*t39.*8.0e+1-Ifris.*mfris.*t3.*t5.*t8.*t37.*1.6e+1-Ifris.*mfris.*t3.*t5.*t9.*t37.*1.92e+2+Ic1.*Ic2.*l3.*m3.*mfris.*rfris.*t2.*1.92e+2-Ic3.*m2.*m3.*mfris.*t3.*t5.*t37.*1.28e+2-Ifris.*m2.*m3.*mfris.*t3.*t5.*t37.*1.28e+2+Ic2.*l3.*m1.*rfris.*t2.*t3.*t12.*6.4e+1+Ic2.*l3.*m2.*rfris.*t2.*t3.*t12.*2.56e+2+Ic1.*l3.*m2.*rfris.*t2.*t5.*t12.*6.4e+1+Ic2.*l3.*m3.*rfris.*t2.*t3.*t12.*4.48e+2+Ic1.*l3.*m3.*rfris.*t2.*t5.*t12.*4.48e+2+Ic2.*l3.*mfris.*rfris.*t2.*t3.*t9.*1.92e+2+Ic1.*l3.*mfris.*rfris.*t2.*t5.*t9.*1.92e+2-Ic2.*l3.*rfris.*t3.*t13.*t26.*t33.*1.28e+2-Ic1.*l3.*rfris.*t5.*t13.*t27.*t34.*1.28e+2+m1.*m2.*m3.*mfris.*t3.*t5.*t7.*8.0+m1.*m2.*m3.*mfris.*t3.*t5.*t15.*4.0+l3.*m1.*rfris.*t2.*t3.*t5.*t13.*6.4e+1+l3.*m2.*rfris.*t2.*t3.*t5.*t13.*3.2e+2+l3.*m3.*rfris.*t2.*t3.*t5.*t13.*7.04e+2+l3.*mfris.*rfris.*t2.*t3.*t5.*t10.*1.92e+2-m2.*m3.*t3.*t5.*t7.*t12.*t37.*2.56e+2-m1.*m3.*t3.*t5.*t7.*t12.*t39.*3.2e+1-m2.*m3.*t3.*t5.*t7.*t12.*t38.*3.2e+1-m2.*m3.*t3.*t5.*t7.*t12.*t39.*1.28e+2-m2.*m3.*t3.*t5.*t12.*t15.*t37.*1.92e+2-m1.*m3.*t3.*t5.*t12.*t15.*t41.*1.6e+1-m2.*m3.*t3.*t5.*t12.*t15.*t40.*1.6e+1-m2.*m3.*t3.*t5.*t12.*t15.*t41.*6.4e+1-m2.*mfris.*t3.*t5.*t7.*t9.*t37.*1.44e+2-m3.*mfris.*t3.*t5.*t7.*t8.*t37.*3.2e+1-m1.*mfris.*t3.*t5.*t7.*t9.*t39.*2.0e+1-m2.*mfris.*t3.*t5.*t7.*t9.*t38.*2.0e+1-m2.*mfris.*t3.*t5.*t7.*t9.*t39.*8.0e+1-m2.*mfris.*t3.*t5.*t9.*t15.*t37.*6.4e+1-m3.*mfris.*t3.*t5.*t8.*t15.*t37.*1.6e+1+l3.*rfris.*t2.*t3.*t5.*t8.*t12.*6.4e+1+l3.*rfris.*t2.*t3.*t5.*t9.*t12.*6.4e+2-l3.*rfris.*t2.*t3.*t5.*t14.*t37.*2.56e+2-l3.*rfris.*t3.*t5.*t14.*t26.*t33.*1.28e+2-l3.*rfris.*t3.*t5.*t14.*t27.*t34.*1.28e+2+t3.*t5.*t7.*t11.*t25.*t26.*t27.*3.2e+1+t3.*t5.*t7.*t14.*t25.*t26.*t27.*1.28e+2+t3.*t5.*t14.*t15.*t25.*t33.*t34.*1.28e+2+Ic2.*l3.*m1.*m3.*mfris.*rfris.*t2.*t3.*4.8e+1+Ic2.*l3.*m2.*m3.*mfris.*rfris.*t2.*t3.*1.92e+2+Ic1.*l3.*m2.*m3.*mfris.*rfris.*t2.*t5.*4.8e+1-Ic2.*l3.*m3.*rfris.*t3.*t12.*t26.*t33.*1.92e+2-Ic1.*l3.*m3.*rfris.*t5.*t12.*t27.*t34.*1.92e+2-Ic2.*l3.*mfris.*rfris.*t3.*t9.*t26.*t33.*6.4e+1-Ic1.*l3.*mfris.*rfris.*t5.*t9.*t27.*t34.*6.4e+1+l3.*m1.*m2.*rfris.*t2.*t3.*t5.*t12.*1.6e+1+l3.*m1.*m3.*rfris.*t2.*t3.*t5.*t12.*1.12e+2+l3.*m2.*m3.*rfris.*t2.*t3.*t5.*t12.*5.6e+2+l3.*m1.*mfris.*rfris.*t2.*t3.*t5.*t9.*4.8e+1+l3.*m2.*mfris.*rfris.*t2.*t3.*t5.*t9.*2.4e+2+l3.*m3.*mfris.*rfris.*t2.*t3.*t5.*t8.*4.8e+1-l3.*m2.*rfris.*t2.*t3.*t5.*t13.*t37.*2.56e+2-l3.*m3.*rfris.*t2.*t3.*t5.*t13.*t37.*7.04e+2-l3.*m2.*rfris.*t3.*t5.*t13.*t26.*t33.*3.2e+1-l3.*m1.*rfris.*t3.*t5.*t13.*t27.*t34.*3.2e+1-l3.*m3.*rfris.*t3.*t5.*t13.*t26.*t33.*3.2e+2-l3.*m2.*rfris.*t3.*t5.*t13.*t27.*t34.*1.28e+2-l3.*m3.*rfris.*t3.*t5.*t13.*t27.*t34.*3.2e+2-l3.*mfris.*rfris.*t2.*t3.*t5.*t10.*t37.*1.92e+2-l3.*mfris.*rfris.*t3.*t5.*t10.*t26.*t33.*6.4e+1-l3.*mfris.*rfris.*t3.*t5.*t10.*t27.*t34.*6.4e+1-l3.*rfris.*t2.*t3.*t5.*t8.*t12.*t37.*6.4e+1-l3.*rfris.*t2.*t3.*t5.*t9.*t12.*t37.*6.4e+2-l3.*rfris.*t3.*t5.*t9.*t12.*t26.*t33.*2.56e+2-l3.*rfris.*t3.*t5.*t9.*t12.*t27.*t34.*2.56e+2+l3.*rfris.*t3.*t5.*t14.*t25.*t26.*t34.*1.28e+2+l3.*rfris.*t3.*t5.*t14.*t25.*t27.*t33.*1.28e+2+m2.*t3.*t5.*t7.*t10.*t25.*t26.*t27.*1.6e+1+m2.*t3.*t5.*t7.*t13.*t25.*t26.*t27.*6.4e+1+m3.*t3.*t5.*t7.*t13.*t25.*t26.*t27.*3.84e+2+m2.*t3.*t5.*t13.*t15.*t25.*t33.*t34.*6.4e+1+m3.*t3.*t5.*t13.*t15.*t25.*t33.*t34.*2.56e+2+mfris.*t3.*t5.*t7.*t10.*t25.*t26.*t27.*1.92e+2+t3.*t5.*t7.*t9.*t12.*t25.*t26.*t27.*4.16e+2+t3.*t5.*t9.*t12.*t15.*t25.*t33.*t34.*1.28e+2+l3.*m1.*m2.*m3.*mfris.*rfris.*t2.*t3.*t5.*1.2e+1-l3.*m2.*m3.*rfris.*t2.*t3.*t5.*t12.*t37.*4.48e+2-l3.*m2.*m3.*rfris.*t3.*t5.*t12.*t26.*t33.*4.8e+1-l3.*m1.*m3.*rfris.*t3.*t5.*t12.*t27.*t34.*4.8e+1-l3.*m2.*m3.*rfris.*t3.*t5.*t12.*t27.*t34.*1.92e+2-l3.*m2.*mfris.*rfris.*t2.*t3.*t5.*t9.*t37.*1.92e+2-l3.*m3.*mfris.*rfris.*t2.*t3.*t5.*t8.*t37.*4.8e+1-l3.*m2.*mfris.*rfris.*t3.*t5.*t9.*t26.*t33.*1.6e+1-l3.*m1.*mfris.*rfris.*t3.*t5.*t9.*t27.*t34.*1.6e+1-l3.*m2.*mfris.*rfris.*t3.*t5.*t9.*t27.*t34.*6.4e+1+l3.*m2.*rfris.*t3.*t5.*t13.*t25.*t26.*t34.*6.4e+1+l3.*m2.*rfris.*t3.*t5.*t13.*t25.*t27.*t33.*6.4e+1+l3.*m3.*rfris.*t3.*t5.*t13.*t25.*t26.*t34.*3.2e+2+l3.*m3.*rfris.*t3.*t5.*t13.*t25.*t27.*t33.*3.2e+2+l3.*mfris.*rfris.*t3.*t5.*t10.*t25.*t26.*t34.*6.4e+1+l3.*mfris.*rfris.*t3.*t5.*t10.*t25.*t27.*t33.*6.4e+1+m2.*m3.*t3.*t5.*t7.*t12.*t25.*t26.*t27.*1.28e+2+m2.*m3.*t3.*t5.*t12.*t15.*t25.*t33.*t34.*6.4e+1+m2.*mfris.*t3.*t5.*t7.*t9.*t25.*t26.*t27.*8.0e+1+l3.*rfris.*t3.*t5.*t9.*t12.*t25.*t26.*t34.*2.56e+2+l3.*rfris.*t3.*t5.*t9.*t12.*t25.*t27.*t33.*2.56e+2+l3.*m2.*m3.*rfris.*t3.*t5.*t12.*t25.*t26.*t34.*9.6e+1+l3.*m2.*m3.*rfris.*t3.*t5.*t12.*t25.*t27.*t33.*9.6e+1+l3.*m2.*mfris.*rfris.*t3.*t5.*t9.*t25.*t26.*t34.*3.2e+1+l3.*m2.*mfris.*rfris.*t3.*t5.*t9.*t25.*t27.*t33.*3.2e+1);
