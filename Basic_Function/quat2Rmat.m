function Rmat = quat2Rmat(quat)

% Convert the input quaternion into rotation matrix 
% Implemented by Gunhee Koo

% Input
% quat = 1x4 vector and its 2-norm is 1

% Output
% Rmat = 3x3 rotation matrix


    r1 = zeros(3,1);
    r2 = r1;
    r3 = r1;

    h0 = quat(1);
    h1 = quat(2);
    h2 = quat(3);
    h3 = quat(4);

    r1(1) = 1 - 2*(h2^2 + h3^2);
    r1(2) = 2*(h1*h2 + h0*h3);
    r1(3) = 2*(h1*h3 - h0*h2);

    r2(1) = 2*(h1*h2 - h0*h3);
    r2(2) = 1 - 2*(h1^2 + h3^2);
    r2(3) = 2*(h2*h3 + h0*h1);

    r3(1) = 2*(h1*h3 + h0*h2);
    r3(2) = 2*(h2*h3 - h0*h1);
    r3(3) = 1 - 2*(h1^2 + h2^2);

    Rmat = [r1, r2, r3];
end