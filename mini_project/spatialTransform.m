function T10 = spatialTransform(DH_params)
% spatialTransform
% Create a spatial transform using standard DH parameters.
%
% Input: DH_params = [alpha_{i-1} (deg), a_{i-1} (m), d_i (m), theta_i (deg)]
% Output: 4x4 homogeneous transform

alp0   = deg2rad(DH_params(1));
a0     = DH_params(2);
d1     = DH_params(3);
theta1 = deg2rad(DH_params(4));

T10 = [cos(theta1)           -sin(theta1)            0           a0; ...
       sin(theta1)*cos(alp0)  cos(theta1)*cos(alp0) -sin(alp0)  -sin(alp0)*d1; ...
       sin(theta1)*sin(alp0)  cos(theta1)*sin(alp0)  cos(alp0)   cos(alp0)*d1; ...
       0                      0                      0           1];
end