function F = PoseTQuatOptimFunc(x, B, h0v, qPtSet1, qPtSet2)

% [Reference Paper] : Schmidt, J., & Niemann, H. (2001, November). Using Quaternions for Parametrizing 3-D Rotations in Unconstrained Nonlinear Optimization. In Vmv (Vol. 1, pp. 399-406).
% Please note that this open source code is an implementation of the study of J.Schmidt and Neimann (VMV workshop 2001) by Gunhee Koo.
% Additionally, this code requires 'Optimization Toolbox' by Matlab for estimation.

% Input
% x : pose set [t(1), t(2), t(3), v(1), v(2), v(3)] , where [v(1),v(2),v(3)] are 3 parameters to represent quaternion qw, qx, qy, qz.

% B : (4x3) matrix for conversion 'v' to 'quaternion' 
% h0v : initial quaternion

% qPtSet1 : mx4 matrix 3d point set in coords{1}. Its 1st column is zero vector.
% qPtSet2 : mx4 matrix 3d point set in coords{2}. Its 1st column is zero vector.




t = [x(1), x(2), x(3)];
qt = [0,t];

v = [x(4); x(5); x(6)];

h0v = [h0v(1), h0v(2), h0v(3), h0v(4)];

v4 = B*v;
theta = norm(v4);
v4n = v4/theta;

v4n = [v4n(1), v4n(2), v4n(3), v4n(4)];

hz = sin(theta)*v4n + cos(theta)*h0v;  % update v until hz is the desired quaternion

qPtSet2Est = quatMultiply(quatMultiply(hz,qPtSet1), quatConjugate(hz)) + qt;

PtSet2 = qPtSet2(:,2:4);
PtSet2Est = qPtSet2Est(:,2:4); 


Err = (PtSet2 - PtSet2Est);
Err = (sum(Err.^2,2).^0.5);

F = Err(:);

