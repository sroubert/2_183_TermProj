function out1 = ddtheta1Lag(Ic1,Ic2,l1,l2,m1,m2,tau1,tau2,th1,th2,th1dot,th2dot)
%DDTHETA1LAG
%    OUT1 = DDTHETA1LAG(IC1,IC2,L1,L2,M1,M2,TAU1,TAU2,TH1,TH2,TH1DOT,TH2DOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    15-Apr-2021 16:17:43

t2 = l1.^2;
t3 = l2.^2;
t4 = m2.^2;
t5 = th2dot.^2;
t6 = -th2;
t7 = t6+th1;
t8 = cos(t7);
t9 = sin(t7);
out1 = ((Ic2.*tau1.*-8.0-m2.*t3.*tau1.*2.0+l1.*l2.*m2.*t8.*tau2.*4.0+l1.*l2.^3.*t4.*t5.*t9+t2.*t3.*t4.*t8.*t9.*th1dot.^2.*2.0+Ic2.*l1.*l2.*m2.*t5.*t9.*4.0).*-2.0)./(Ic1.*Ic2.*1.6e+1+Ic2.*m1.*t2.*4.0+Ic1.*m2.*t3.*4.0+Ic2.*m2.*t2.*1.6e+1+t2.*t3.*t4.*4.0+m1.*m2.*t2.*t3-t2.*t3.*t4.*t8.^2.*4.0);
