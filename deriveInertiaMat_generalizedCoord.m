function [inertia_matrix] = deriveInertiaMat_generalizedCoord(jacob, massMatrix)

inertia_matrix = ( jacob'*massMatrix*jacob );

end