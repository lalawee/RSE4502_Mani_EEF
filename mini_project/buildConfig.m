function [robot, DH_table, jointTypes, jLimits, vLimits, q_hint] = buildConfig(configStr)
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
%   jLimits    - mx2 joint limits [min, max] for active joints only
%   vLimits    - mx1 max joint velocity magnitudes (rad/s or m/s)
%   q_hint     - mx1 rough posture hint for initial config search
%
% DH Convention: Modified DH (Craig)

switch upper(configStr)

    % =====================================================================
    case 'JETARM'
    % =====================================================================
    scale = 6;
    l0 = 0.10315 * scale;
    l1 = 0.12942 * scale;
    l2 = 0.12942 * scale;
    l3 = 0.05945 * scale;
    l4 = 0.02545 * scale;

    fprintf('JetArm scaled %dx link lengths:\n', scale);
    fprintf('  l0=%.3fm, l1=%.3fm, l2=%.3fm, l3=%.3fm, l4=%.3fm\n\n', ...
            l0, l1, l2, l3, l4);

    DH_cell = {
         0,   0,   0,    0,    'P';
         0,   0,   l0,   0,    'R';
       -90,   0,   0,   -90,   'R';
         0,   l1,  0,    0,    'R';
         0,   l2,  0,   -90,   'R';
       -90,   0,   l3,   0,    'R';
         0,   0,   l4,   180,  'F';
    };

    jLimits = [
        -0.5,  1.5;      % Joint 1: prismatic
        -2*pi, 2*pi;     % Joint 2: base yaw
        -2*pi, 2*pi;     % Joint 3: shoulder
        -pi,   pi;       % Joint 4: elbow
        -pi,   pi;       % Joint 5: wrist pitch
        -pi,   pi;       % Joint 6: wrist roll
    ];

    vLimits = [10; 10; 10; 10; 10; 10];

    % Posture hint: prismatic at 1.0m, yaw toward seam, elbow bent
    q_hint = [1.0; pi; -1.571; -2.646; -0.175; 0.0];

    % =====================================================================
    case 'RRP'
    % =====================================================================
    d_base = 1.0;

    DH_cell = {
         0,   0,   d_base,  0,   'R';
        90,   0,   0,       0,   'R';
         0,   0,   0,       0,   'P';
    };

    jLimits = [
       -pi,   pi;
       -pi/2, pi/2;
        0.1,  0.8;
    ];

    vLimits = [
        1.0;
        1.0;
        0.3;
    ];

    q_hint = [pi; 0.0; 0.3];

    % =====================================================================
    case 'RRR'
    % =====================================================================
    d_base = 0.3;
    L2     = 0.8;
    L3     = 0.7;

    DH_cell = {
         0,   0,    d_base,  0,   'R';
        90,   0,    0,       0,   'R';
         0,   L2,   0,       0,   'R';
         0,   L3,   0,       0,   'F';
    };

    jLimits = [
       -pi,   pi;
       -pi/2, pi/2;
       -pi,   pi;
    ];

    vLimits = [
        1.0;
        1.0;
        1.5;
    ];

    q_hint = [pi; 0.0; -pi/2];

    % =====================================================================
    otherwise
        error('Unknown config "%s". Available: JetArm, RRP, RRR', configStr);
end

% Build robot
robot      = buildRobotFromDH(DH_cell, configStr);
jointTypes = DH_cell(:, 5);
DH_table   = cell2mat(DH_cell(:, 1:4));

% Apply joint limits with three-step fix to avoid warning
warning('off', 'robotics:robotics:rigidBodyJoint:HomePositionOutOfLimits');
activeIdx = 0;
for i = 1:length(robot.Bodies)
    jt = robot.Bodies{i}.Joint.Type;
    if ~strcmp(jt, 'fixed')
        activeIdx = activeIdx + 1;
        robot.Bodies{i}.Joint.PositionLimits = [-1e6, 1e6];
        robot.Bodies{i}.Joint.HomePosition   = mean(jLimits(activeIdx, :));
        robot.Bodies{i}.Joint.PositionLimits = jLimits(activeIdx, :);
    end
end
warning('on', 'robotics:robotics:rigidBodyJoint:HomePositionOutOfLimits');

fprintf('Built robot: %s (%d bodies, %d active joints)\n', ...
        configStr, size(DH_table,1), sum(~strcmpi(jointTypes,'F')));
showdetails(robot);

end