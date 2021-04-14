function [inertia_matrix] = deriveInertiaMat_generalizedCoord(jacob, massMatrix)

inertia_matrix = simplify( jacob'*massMatrix*jacob );

end