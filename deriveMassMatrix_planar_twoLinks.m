function [M_matrix] = deriveMassMatrix_planar_twoLinks()

syms m1 Ic1 m2 Ic2

M_matrix = diag( [m1 m1 Ic1 m2 m2 Ic2] );

end