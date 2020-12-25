function pq = quatMultiply(pSet,qSet)

% Input Variables
% pSet : a quaternion 1x4  which is composed of pw, px, py, pz  
% qSet : a quaternion 1x4 vector which is composed of qw, qx, qy, qz

% cf. Only one of input variables (p or q) could be a set of m quaternions in the form of (mx4) matrix

% Output Variable
% pq = p*q, result of quaternion multiplication , a quaternion 1x4 vector

% cf. when a input variable is mx4 matrix, the result pq is also a mx4 matrix



pnum = size(pSet,1);
qnum = size(qSet,1);

if pnum > 1
    if qnum > 1
        disp('Error by Input variables in the funtion quatMultiply'); 
        pq = [0,0,0,0];
        
    else
        % mat * vector
        pq = [];
        q = qSet;
        for i = 1:pnum
            p = pSet(i,:);
            
            p1 = p(1);
            p2 = p(2);
            p3 = p(3);
            p4 = p(4);

            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            q4 = q(4);

            pq1 = p1*q1 - p2*q2 - p3*q3 - p4*q4;
            pq2 = p1*q2 + p2*q1 + p3*q4 - p4*q3;
            pq3 = p1*q3 - p2*q4 + p3*q1 + p4*q2;
            pq4 = p1*q4 + p2*q3 - p3*q2 + p4*q1;
            
            pq = [pq;[pq1,pq2,pq3,pq4]];
        end
    end
else
    if qnum > 1
        % vector * matrix
        pq = [];
        p = pSet;
        for i = 1:qnum
            q = qSet(i,:);
            
            p1 = p(1);
            p2 = p(2);
            p3 = p(3);
            p4 = p(4);

            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            q4 = q(4);

            pq1 = p1*q1 - p2*q2 - p3*q3 - p4*q4;
            pq2 = p1*q2 + p2*q1 + p3*q4 - p4*q3;
            pq3 = p1*q3 - p2*q4 + p3*q1 + p4*q2;
            pq4 = p1*q4 + p2*q3 - p3*q2 + p4*q1;
            
            pq = [pq;[pq1,pq2,pq3,pq4]];
        end
    else
        % vector * vector
        p = pSet;
        q = qSet;
        
        p1 = p(1);
        p2 = p(2);
        p3 = p(3);
        p4 = p(4);

        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q4 = q(4);

        pq1 = p1*q1 - p2*q2 - p3*q3 - p4*q4;
        pq2 = p1*q2 + p2*q1 + p3*q4 - p4*q3;
        pq3 = p1*q3 - p2*q4 + p3*q1 + p4*q2;
        pq4 = p1*q4 + p2*q3 - p3*q2 + p4*q1;

        pq = [pq1, pq2, pq3, pq4];
        
    end
end



