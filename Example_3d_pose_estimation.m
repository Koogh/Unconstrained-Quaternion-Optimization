% This is an example code to show the non-constraint optimization of quaternion using 1x3 vector following the study of J.Schmidt and Neimann. 
% [Reference Paper] : Schmidt, J., & Niemann, H. (2001, November). Using Quaternions for Parametrizing 3-D Rotations in Unconstrained Nonlinear Optimization. In Vmv (Vol. 1, pp. 399-406).

% In this example, we try to estimate the 3D transformation parameter including quaternion (1x4 vector) and translation (1x3 vector) using two 3d point sets.
% Before optimization, the (1x4) quaternion is converted into a (1x3) vector, so that we practically estimate a (1x3) vector and translation (1x3) vector.
% After optimization, the (1x3) vector is converted into the (1x4) quaternion again, so that we get the estimated quaternion (1x4 vector) and translation (1x3 vector).

% Please note that this open source code is an implementation of the study of J.Schmidt and Neimann (VMV workshop 2001) by Gunhee Koo.
% Additionally, this code requires 'Optimization Toolbox' by Matlab for Optimization.



%% Initialize
clc; clear all; close all;

addpath('Basic_Function');
addpath('Error_Function');

%% [Setting] Define Coord {1} such as Origin
t1 = [0; 0; 0];
q1 = [1, 0, 0, 0];
q1 = q1/norm(q1);

%% [Setting] Get the m measured 3d points set in coords {1} : mx3 matrix
Pt_c1 = randi(100,15,3)-50;

qPt_c1 = [zeros(size(Pt_c1,1),1), Pt_c1];  % 3d point set in quaternion form


%% [Reference] Transformation between Coord {1} and Coord {2}  
% t12 & q12 would be the target 3d transformation (3d pose) to be estimated.
q12 = randi(100, 1,4)-50;
q12 = q12./norm(q12);
if q12(1) < 0 
    q12 = -q12;
end

t12 = randi(100,3,1)-50;
t12 = t12/100;

disp(strcat('Set the reference relative quaternion q12 as [', num2str(q12),']'))
disp(strcat('Set the reference relative translation t12 as [', num2str(t12'), ']'))

    

%% Transform the 3d point set in coords {1} into the point set in coords {2}
% [Case 1] Rotate by quaterion
qPt_c2_ver_quat = quatMultiply(quatMultiply(q12, qPt_c1), quatConjugate(q12));
qPt_c2_ver_quat = qPt_c2_ver_quat + [0,t12(1),t12(2), t12(3)];

Pt_c2_ver_quat = qPt_c2_ver_quat(:,2:end);


% [Case 2] Rotate by rotation matrix
R12 = quat2Rmat(q12);

Pt_c2_ver_Rmat = (R12*Pt_c1' + t12)'; 


bool_ref_transform = abs(Pt_c2_ver_quat-Pt_c2_ver_Rmat);
bool_ref_transform = sum(bool_ref_transform(:)) < 0.0001;
if bool_ref_transform 
    disp('INFO :: Pt_c2_ver_quat and Pt_c2_ver_Rmat are equal, so that we can transform the 3d points only by quaternion operations.')
    qPt_c2 = qPt_c2_ver_quat;
end


%% Estimate the 3D Transformation (quaternion & t) using both the 3d points in coords {1} and 3d points in coords {2} 
h0v = q1;         % initial quaternion 
n = h0v;

% Set x as Random Values, but it satisfies nx = 1
x = zeros(1,4);  
x(2) = randi(100)/100;
x(3) = randi(100)/100;
x(4) = randi(100)/100;

x(1) = (1/n(1))*(1-n(2)*x(2)-n(3)*x(3)-n(4)*x(4));

a = [1/n(1), 0, 0, 0]';
bp1 = [-n(2)/n(1), 1, 0, 0]';
bp2 = [-n(3)/n(1), 0, 1, 0]';
bp3 = [-n(4)/n(1), 0, 0, 1]';

Bp = [bp1, bp2, bp3];

[B,D,V] = svd(Bp,'econ');

% Initialize the target parameters 
vinit = [0.01, 0.01, 0.01];                            % initial v
tinit = t12-(randi(10,size(t12))/20);                  % initial t

xinit = [tinit(1), tinit(2), tinit(3), vinit(1), vinit(2), vinit(3)];

% Optimize
fun1 = @(x)PoseTQuatOptimFunc(x,B, h0v ,qPt_c1, qPt_c2);

options = optimoptions('lsqnonlin','Display', 'iter');
options.Algorithm = 'levenberg-marquardt';

xEst = lsqnonlin(fun1, xinit, [],[],options);

%% Estimated quaternion & translation
v4Est = B*[xEst(4);xEst(5);xEst(6)];
thetaEst = norm(v4Est);
v4nEst = v4Est/thetaEst;

v4nEst = [v4nEst(1), v4nEst(2), v4nEst(3), v4nEst(4)];


q12Est = sin(thetaEst)*v4nEst + cos(thetaEst)*h0v;
if q12Est(1) < 0 
    q12Est = -q12Est;
end
t12Est = [xEst(1); xEst(2); xEst(3)];

%% Show the Estimation Result (quaternion & translation)
disp('================================ RESULT ========================================');
disp(strcat('Reference relative quaternion q12 = [', num2str(q12), ']'))
disp(strcat('Reference relative translation t12 = [', num2str(t12'), ']'))

disp(strcat('Estimated relative quaternion q12 = [', num2str(q12Est), ']'))
disp(strcat('Estimated relative translation t12 = [', num2str(t12Est'), ']'))