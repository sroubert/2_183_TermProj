function [M_matrix] = deriveMassMatrix_planar_twoDOF_fris(m1, Ic1, m2, Ic2,...
    l1, l2, ...
    mfris, Ifris, rfris, thFrisOrient)


rf_2 = sqrt ( rfris^2 + l2^2 - 2*rfris*l2*cos(pi-thFrisOrient) );

M_matrix = diag( [m1, m1, Ic1, ...
    m2+mfris, m2+mfris, Ic2 + Ifris + mfris*rf_2^2] );

end