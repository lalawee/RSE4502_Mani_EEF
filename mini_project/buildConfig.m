function [robot, DH_table, jointTypes] = buildConfig(configStr)
% buildConfig
% Builds a robot model from a named configuration string.
%
% Input:
%   configStr - e.g. 'JetArm' (scaled 6x), extensible for other configs
%
% Output:
%   robot      - rigidBodyTree object
%   DH_table   - nx4 numeric matrix [alpha_deg, a_m, d_m, theta_offset_deg]
%   jointTypes - nx1 cell array of 'R', 'P', or 'F'
%
% DH Convention: Modified DH (Craig)
% Row i: [alpha_{i-1}, a_{i-1}, d_i, theta_offset_i, jointType]
%
% Robot base is offset at (1.0, 0, 0) from cylinder center (world origin)
% Welding target: r=0.5m, z=1.0m around cylinder at world origin

switch upper(configStr)

    % =====================================================================
    case 'JETARM'
    % =====================================================================
    % Scaled JetArm 6x — RRRRR + fixed tool frame
    % Original JetArm link lengths scaled by 6 for welding task reach
    %
    % Joint 1 (R): Base rotation — yaw about world Z
    % Joint 2 (R): Shoulder — pitch up/down
    % Joint 3 (R): Elbow — pitch up/down
    % Joint 4 (R): Wrist pitch
    % Joint 5 (R): Wrist roll
    % Joint 6 (F): Fixed tool frame
    %
    % Base offset: robot base placed at (1.0, 0, 0) from cylinder center
    % This is handled in the welding path definition (task space offset)

    scale = 6;
    l0 = 0.10315 * scale;   % 0.618m - base height
    l1 = 0.12942 * scale;   % 0.776m - upper arm
    l2 = 0.12942 * scale;   % 0.776m - forearm
    l3 = 0.05945 * scale;   % 0.357m - wrist
    l4 = 0.02545 * scale;   % 0.153m - tool


    fprintf('JetArm scaled %dx link lengths:\n', scale);
    fprintf('  l0=%.3fm, l1=%.3fm, l2=%.3fm, l3=%.3fm, l4=%.3fm\n\n', l0,l1,l2,l3,l4);

    % [alpha_deg, a_m, d_m, theta_offset_deg, jointType]
    DH_cell = {
         0,   0,    l0,   0,     'R';   % Joint 1: base yaw
       -90,   0,    0,   -90,    'R';   % Joint 2: shoulder pitch
         0,   l1,   0,    0,     'R';   % Joint 3: elbow pitch
         0,   l2,   0,   -90,    'R';   % Joint 4: wrist pitch
       -90,   0,    l3,   0,     'R';   % Joint 5: wrist roll
         0,   0,    l4,   180,   'F';   % Joint 6: fixed tool frame
    };

    % =====================================================================
    % Add new configurations below following the same pattern:
    % case 'MYROBOT'
    %   DH_cell = { ... };
    % =====================================================================

    otherwise
        error('Unknown config "%s". Available: JetArm', configStr);
end

% Build robot and extract outputs — same for all cases
robot      = buildRobotFromDH(DH_cell, configStr);
jointTypes = DH_cell(:, 5);
DH_table   = cell2mat(DH_cell(:, 1:4));

fprintf('Built robot: %s (%d bodies)\n', configStr, size(DH_table, 1));
showdetails(robot);

end