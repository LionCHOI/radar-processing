function rotMat = getRotationMatrixFromVector(q)
% compute 3x3 rotation matrix from quaternion 
% orientation of GCS relative to LCS (Android system)
% q = [cos(theta/2), ex sin(theta/2), ey sin(theta/2), ez sin(theta/2)]

if numel(q) ~= 4
    error('Dimension error');
end

q = q / sqrt(sum(q.^2));

%% Rotation matrix from quaternion

sq_q1 = 2 * q(2)^2;
sq_q2 = 2 * q(3)^2;
sq_q3 = 2 * q(4)^2;
q1_q2 = 2 * q(2) * q(3);
q3_q0 = 2 * q(4) * q(1);
q1_q3 = 2 * q(2) * q(4);
q2_q0 = 2 * q(3) * q(1);
q2_q3 = 2 * q(3) * q(4);
q1_q0 = 2 * q(2) * q(1);

rotMat = [1 - sq_q2 - sq_q3, q1_q2 - q3_q0, q1_q3 + q2_q0;...
         q1_q2 + q3_q0, 1 - sq_q1 - sq_q3, q2_q3 - q1_q0;...
         q1_q3 - q2_q0, q2_q3 + q1_q0, 1-sq_q1 - sq_q2];
     
%% EOFD