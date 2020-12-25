function qc = quatConjugate(q)

% Convert the input quaternion into its conjugate quaternion
% Implemented by Gunhee Koo

% Input Variable
% q : 1x4 vector composed of qw, qx, qy, and qz.

% Output Variable
% qc : 1x4 vector composed of qw, qx, qy, and qz.

    
    qc = [q(1), -q(2), -q(3), -q(4)];
   
    
end