function [kineticEnergy] = deriveKineticEnergy(omega, I_mat)
%omega is d/dt of generalized coordinates, not velocity

kineticEnergy = simplify ( 0.5*omega*I_mat*omega' );

end