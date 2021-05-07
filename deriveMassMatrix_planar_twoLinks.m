function [M_matrix] = deriveMassMatrix_planar_twoLinks(m1, Ic1, m2)

syms m1 Ic1 m2 Ic2 real

M_matrix = diag( [m1 m1 Ic1 m2 m2 Ic2] );

end