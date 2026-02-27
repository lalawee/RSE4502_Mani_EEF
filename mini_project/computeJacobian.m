function J = computeJacobian(DH_table, q, jointTypes)
% computeJacobian
% Computes the geometric Jacobian from first principles via direct column
% construction. Works for any combination of revolute and prismatic joints.
%
% Inputs:
%   DH_table   - nx4 numeric matrix [alpha_deg, a_m, d_m, theta_offset_deg]
%   q          - mx1 current joint values (rad for R, m for P)
%                where m = number of non-fixed joints
%   jointTypes - nx1 cell array {'R','P','F'} for each DH row
%
% Output:
%   J          - 6xm Jacobian matrix [angular (3xm); linear (3xm)]
%
% Convention (Craig's Modified DH):
%   For each active joint i, z_i and p_i are extracted from T_0^i
%   (the transform AFTER the joint's alpha twist and offset are applied).
%
%   Revolute:  J_i = [z_i; z_i x (p_e - p_i)]
%   Prismatic: J_i = [0;   z_i               ]
%   Fixed:     skipped — no contribution to EE velocity

n = size(DH_table, 1);   % total DH rows including fixed

% Count active (non-fixed) joints
activeIdx = find(~strcmpi(jointTypes, 'F'));
m = length(activeIdx);   % should match length(q)

if length(q) ~= m
    error('computeJacobian: q has %d elements but %d active joints found.', ...
          length(q), m);
end

% --- Step 1: Compute all FK transforms T_0^i for each DH row ---
T = eye(4);
Transforms = cell(n+1, 1);
Transforms{1} = T;   % T_0^0 = identity

qIdx = 0;    % index into q — increments only for active (non-fixed) joints

for i = 1:n
    alpha = DH_table(i, 1);
    a     = DH_table(i, 2);
    d     = DH_table(i, 3);
    theta = DH_table(i, 4);

    jt = upper(char(jointTypes{i}));

    if strcmp(jt, 'F')
        % Fixed joint — just apply the constant transform
        dh_row = [alpha, a, d, theta];
    elseif strcmp(jt, 'R')
        % Revolute — q adds to theta
        qIdx = qIdx + 1;
        dh_row = [alpha, a, d, theta + rad2deg(q(qIdx))];
    else
        % Prismatic — q adds to d
        qIdx = qIdx + 1;
        dh_row = [alpha, a, d + q(qIdx), theta];
    end

    T = T * spatialTransform(dh_row);
    Transforms{i+1} = T;
end

% --- Step 2: End effector position ---
p_e = Transforms{n+1}(1:3, 4);

% --- Step 3: Build Jacobian column by column ---
% For Craig's modified DH, z_i and p_i are taken from T_0^i
% (i.e. Transforms{i+1}), which is the frame AFTER the joint acts.

J    = zeros(6, m);
qIdx = 0;

%for i = 1:n
%    fprintf('Frame %d: z=[%.3f,%.3f,%.3f] p=[%.3f,%.3f,%.3f]\n', ...
%        i-1, Transforms{i}(1:3,3)', Transforms{i}(1:3,4)');
%end
%fprintf('EE: p=[%.3f,%.3f,%.3f]\n', p_e');

for i = 1:n
    jt = upper(char(jointTypes{i}));
    if strcmp(jt, 'F'), continue; end

    qIdx = qIdx + 1;

    % T_prev = Transforms{i};        % T_0^{i-1}
    % z      = T_prev(1:3, 3);       % z-hat_{i-1}: 3rd column of rotation
    % p      = T_prev(1:3, 4);       % origin of frame i-1
    T_curr = Transforms{i+1};     % frame after joint i
    
    z = T_curr(1:3, 3);           % z-axis AFTER alpha twist
    p = T_curr(1:3, 4);           % origin of frame i

    if strcmp(jt, 'R')
        J(1:3, qIdx) = z;                    % angular contribution
        J(4:6, qIdx) = cross(z, p_e - p);   % linear contribution
    else % prismatic
        J(1:3, qIdx) = zeros(3,1);           % no angular contribution
        J(4:6, qIdx) = z;                    % linear along z axis
    end
end

end