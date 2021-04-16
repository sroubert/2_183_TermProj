function out1 = ddtheta2Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot)
%DDTHETA2LAG
%    OUT1 = DDTHETA2LAG(IC1,IC2,L1,L2,M1,M2,TAU1,TAU2,TH1,TH2,TH1DOT,TH2DOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    15-Apr-2021 16:17:43

t2 = l1.^2;
t3 = l1.^3;
t4 = l2.^2;
t5 = m2.^2;
t6 = th1dot.^2;
t7 = -th2;
t8 = t7+th1;
t9 = cos(t8);
t10 = sin(t8);
out1 = ((Ic1.*tau2.*8.0+m1.*t2.*tau2.*2.0+m2.*t2.*tau2.*8.0-l1.*l2.*m2.*t9.*tau1.*4.0+l2.*t3.*t5.*t6.*t10.*4.0+l2.*m1.*m2.*t3.*t6.*t10+t2.*t4.*t5.*t9.*t10.*th2dot.^2.*2.0+Ic1.*l1.*l2.*m2.*t6.*t10.*4.0).*2.0)./(Ic1.*Ic2.*1.6e+1+Ic2.*m1.*t2.*4.0+Ic2.*m2.*t2.*1.6e+1+Ic1.*m2.*t4.*4.0+t2.*t4.*t5.*4.0+m1.*m2.*t2.*t4-t2.*t4.*t5.*t9.^2.*4.0);
