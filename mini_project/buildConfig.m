function [robot, DH_table, jointTypes, jLimits] = buildConfig(configStr)
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
%   jLimits    - mx2 joint limits [min, max] for active joints only (m=non-fixed)
%
% DH Convention: Modified DH (Craig)

switch upper(configStr)

    % =====================================================================
    case 'JETARM_NOPRIS'
    % =====================================================================
    % Scaled JetArm 6x — RRRRR + fixed tool frame
    % Original configuration without prismatic joint
    
    scale = 6;
    l0 = 0.10315 * scale;
    l1 = 0.12942 * scale;
    l2 = 0.12942 * scale;
    l3 = 0.05945 * scale;
    l4 = 0.02545 * scale;
    
    fprintf('JetArm (no prismatic) scaled %dx link lengths:\n', scale);
    fprintf('  l0=%.3fm, l1=%.3fm, l2=%.3fm, l3=%.3fm, l4=%.3fm\n\n', ...
            l0, l1, l2, l3, l4);
    
    DH_cell = {
         0,   0,    l0,   0,     'R';   % Joint 1: base yaw
       -90,   0,    0,   -90,    'R';   % Joint 2: shoulder pitch
         0,   l1,   0,    0,     'R';   % Joint 3: elbow pitch
         0,   l2,   0,   -90,    'R';   % Joint 4: wrist pitch
       -90,   0,    l3,   0,     'R';   % Joint 5: wrist roll
         0,   0,    l4,   180,   'F';   % Fixed tool frame
    };
    
    jLimits = [
       -pi,    pi;     % Joint 1: base yaw
       -pi/2,  pi/2;   % Joint 2: shoulder pitch
       -pi,    pi;     % Joint 3: elbow pitch
       -pi/2,  pi/2;   % Joint 4: wrist pitch
       -pi,    pi;     % Joint 5: wrist roll
    ];


    % =====================================================================
    case 'JETARM'
    % =====================================================================
    % Scaled JetArm 6x — PRRRRR + fixed tool frame
    % Prismatic joint for vertical lift + 5 revolute joints

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

    % Joint limits [min, max] for active joints only
    % Order matches DH_cell active rows (non-fixed)
    jLimits = [
        0.5,   1.5;    % Joint 1: prismatic lift (m)
       -pi,    pi;     % Joint 2: base yaw (rad)
       -pi/2,  pi/2;   % Joint 3: shoulder pitch (rad)
       -pi,    pi;     % Joint 4: elbow pitch (rad)
       -pi/2,  pi/2;   % Joint 5: wrist pitch (rad)
       -pi,    pi;     % Joint 6: wrist roll (rad)
    ];

    % =====================================================================
    case 'RRP'
    % =====================================================================
    % Simple 3-DOF: Revolute-Revolute-Prismatic
    % Joint 1 (R): Base yaw about world Z
    % Joint 2 (R): Shoulder pitch
    % Joint 3 (P): Radial extension

    d_base = 1.0;   % fixed base height (m)

    DH_cell = {
         0,   0,   d_base,  0,   'R';   % Joint 1: yaw
        90,   0,   0,       0,   'R';   % Joint 2: pitch
         0,   0,   0,       0,   'P';   % Joint 3: prismatic extend
    };

    jLimits = [
       -pi,   pi;     % Joint 1: yaw (rad)
       -pi/2, pi/2;   % Joint 2: pitch (rad)
        0.1,  0.8;    % Joint 3: prismatic extend (m)
    ];

    % =====================================================================
    case 'RRR'
    % =====================================================================
    % Simple 3-DOF all-revolute
    % Joint 1 (R): Base yaw
    % Joint 2 (R): Shoulder pitch
    % Joint 3 (R): Elbow pitch

    d_base = 0.3;
    L2     = 0.8;
    L3     = 0.7;

    DH_cell = {
         0,   0,    d_base,  0,   'R';   % Joint 1: yaw
        90,   0,    0,       0,   'R';   % Joint 2: shoulder pitch
         0,   L2,   0,       0,   'R';   % Joint 3: elbow pitch
         0,   L3,   0,       0,   'F';   % Fixed tool frame
    };

    jLimits = [
       -pi,   pi;     % Joint 1: yaw (rad)
       -pi/2, pi/2;   % Joint 2: shoulder (rad)
       -pi,   pi;     % Joint 3: elbow (rad)
    ];

    % =====================================================================
    otherwise
        error('Unknown config "%s". Available: JetArm, RRP, RRR', configStr);
end

% Build robot and extract outputs
robot      = buildRobotFromDH(DH_cell, configStr);
jointTypes = DH_cell(:, 5);
DH_table   = cell2mat(DH_cell(:, 1:4));

% Apply joint limits to robot bodies
activeIdx = 0;
for i = 1:length(robot.Bodies)
    jt = robot.Bodies{i}.Joint.Type;
    if ~strcmp(jt, 'fixed')
        activeIdx = activeIdx + 1;
        robot.Bodies{i}.Joint.PositionLimits = jLimits(activeIdx, :);
    end
end

fprintf('Built robot: %s (%d bodies, %d active joints)\n', ...
        configStr, size(DH_table,1), sum(~strcmpi(jointTypes,'F')));
showdetails(robot);

end