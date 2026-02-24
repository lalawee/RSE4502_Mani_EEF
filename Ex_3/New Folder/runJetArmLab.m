clc; clear; close all;

%% Link lengths (meters) from the sheet
l0 = 0.10315;
l1 = 0.12942;
l2 = 0.12942;
l3 = 0.05945;
l4 = 0.02545;

%% DH table from your image
% Columns: [alpha_{i-1} deg, a_{i-1} m, d_i m, theta_offset deg, jointType]
%
% Note: rows 1-5 are revolute joints with theta offsets embedded.
% Row 6 is fixed (tool/end frame) with theta = 180 deg.
DH = { ...
     0      0     l0     0      'R';  % i=1
   -90      0      0    -90     'R';  % i=2
     0      l1     0     0      'R';  % i=3
     0      l2     0    -90     'R';  % i=4
   -90      0     l3     0      'R';  % i=5
     0      0     l4    180     'F'}; % i=6

%% Build robot dynamically
[robot, bodyNames] = buildRobotFromDH(DH, "JetArm");
showdetails(robot);

% Choose what frames correspond to S and H for your lab
% Typical guess:
%   S = after joint 5 (link5)
%   H = tool/end (link6)
S_body = "link5";
H_body = "link6";

%% Define test cases (degrees) for [q1 q2 q3 q4 q5]
cases_deg = { ...
    [0   0   0    0    0], ...
    [10 -20  0   20  -45], ...
    [10 -45 30  -45   45] ...
};

%% Run cases
figure;
for k = 1:numel(cases_deg)
    q_deg = cases_deg{k};
    q_rad = deg2rad(q_deg);

    % ---- visualize
    clf;
    updateRobotConfiguration(robot, q_rad);
    drawnow;

    % ---- FK using getTransform
    cfg = homeConfiguration(robot);
    for i = 1:min(length(cfg), length(q_rad))
        cfg(i).JointPosition = q_rad(i);
    end

    T0S = getTransform(robot, cfg, S_body, robot.BaseName);
    T0H = getTransform(robot, cfg, H_body, robot.BaseName);

    fprintf('\n=============================\n');
    fprintf('Case %d: q(deg) = [%s]\n', k, num2str(q_deg));
    fprintf('--- T0S (base -> %s) ---\n', S_body);
    disp(T0S);
    fprintf('--- T0H (base -> %s) ---\n', H_body);
    disp(T0H);
end