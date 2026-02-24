function [robot, DH_table, jointTypes] = buildConfig(configStr)
% buildConfig
% Builds a robot model from a named configuration string.
%
% Input:
%   configStr - 'JetArm', 'RRP', 'RRR'
%
% Output:
%   robot      - rigidBodyTree object
%   DH_table   - nx4 numeric matrix [alpha_deg, a_m, d_m, theta_offset_deg]
%   jointTypes - nx1 cell array of 'R', 'P', or 'F'
%
% DH Convention: Modified DH (Craig)

switch upper(configStr)

    % =====================================================================
    case 'JETARM'
    % =====================================================================
    % Scaled JetArm 6x — PRRRRR + fixed tool frame
    % Prismatic joint added for vertical lift
    % Robot base at (0.8, 0) from cylinder center

    scale = 6;
    l0 = 0.10315 * scale;   % 0.619m - base height
    l1 = 0.12942 * scale;   % 0.777m - upper arm
    l2 = 0.12942 * scale;   % 0.777m - forearm
    l3 = 0.05945 * scale;   % 0.357m - wrist
    l4 = 0.02545 * scale;   % 0.153m - tool

    fprintf('JetArm scaled %dx link lengths:\n', scale);
    fprintf('  l0=%.3fm, l1=%.3fm, l2=%.3fm, l3=%.3fm, l4=%.3fm\n\n', ...
            l0, l1, l2, l3, l4);

    DH_cell = {
         0,   0,   0,    0,    'P';   % Joint 1: prismatic lift (Z)
         0,   0,   l0,   0,    'R';   % Joint 2: base yaw
       -90,   0,   0,   -90,   'R';   % Joint 3: shoulder pitch
         0,   l1,  0,    0,    'R';   % Joint 4: elbow pitch
         0,   l2,  0,   -90,   'R';   % Joint 5: wrist pitch
       -90,   0,   l3,   0,    'R';   % Joint 6: wrist roll
         0,   0,   l4,   180,  'F';   % Fixed tool frame
    };

    % =====================================================================
    case 'RRP'
    % =====================================================================
    % Simple 3-DOF robot: Revolute-Revolute-Prismatic
    % Joint 1 (R): Base yaw about world Z
    % Joint 2 (R): Shoulder pitch
    % Joint 3 (P): Radial extension
    % Robot base at (0.8, 0) from cylinder center

    d_base = 1.0;   % fixed base height (m)

    DH_cell = {
         0,   0,   d_base,  0,   'R';   % Joint 1: yaw
        90,   0,   0,       0,   'R';   % Joint 2: pitch (z2 horizontal)
         0,   0,   0,       0,   'P';   % Joint 3: prismatic extend
    };

    % =====================================================================
    case 'RRR'
    % =====================================================================
    % Simple 3-DOF all-revolute robot
    % Joint 1 (R): Base yaw
    % Joint 2 (R): Shoulder pitch
    % Joint 3 (R): Elbow pitch
    % Robot base at (0.8, 0) from cylinder center

    d_base = 0.3;
    L2     = 0.8;
    L3     = 0.7;

    DH_cell = {
         0,   0,    d_base,  0,   'R';   % Joint 1: yaw
        90,   0,    0,       0,   'R';   % Joint 2: shoulder pitch
         0,   L2,   0,       0,   'R';   % Joint 3: elbow pitch
         0,   L3,   0,       0,   'F';   % Fixed tool frame
    };

    % =====================================================================
    otherwise
        error('Unknown config "%s". Available: JetArm, RRP, RRR', configStr);
end

% Build robot and extract outputs
robot      = buildRobotFromDH(DH_cell, configStr);
jointTypes = DH_cell(:, 5);
DH_table   = cell2mat(DH_cell(:, 1:4));

fprintf('Built robot: %s (%d bodies, %d active joints)\n', ...
        configStr, size(DH_table,1), sum(~strcmpi(jointTypes,'F')));
showdetails(robot);

end