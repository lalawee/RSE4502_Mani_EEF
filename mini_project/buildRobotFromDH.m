function [robot, bodyNames] = buildRobotFromDH(DH_table, robotName)
% buildRobotFromDH
% DH_table can be:
%   - numeric matrix (no joint type column), OR
%   - cell array with last column joint type {'R','P','F'}
%
% Expected columns (first 4):
%   [alpha_deg, a_m, d_m, theta_offset_deg]

if nargin < 2
    robotName = "robot";
end

% Number of rows
n = size(DH_table, 1);

robot = rigidBodyTree('DataFormat','struct','MaxNumBodies',n);
robot.Gravity = [0 0 -9.81];
robot.BaseName = char(robotName);

bodyNames = cell(n,1);
parentName = robot.BaseName;

for i = 1:n
    % ---- Extract row data safely ----
    if iscell(DH_table)
        alpha = DH_table{i,1};
        a     = DH_table{i,2};
        d     = DH_table{i,3};
        th0   = DH_table{i,4};
        jt    = DH_table{i,5};
    else
        % numeric DH with no joint type column -> assume revolute
        alpha = DH_table(i,1);
        a     = DH_table(i,2);
        d     = DH_table(i,3);
        th0   = DH_table(i,4);
        jt    = 'R';
    end

    % Force numeric for the first 4 entries (important!)
    alpha = double(alpha);
    a     = double(a);
    d     = double(d);
    th0   = double(th0);

    jt = upper(char(jt));

    bodyName  = sprintf('link%d', i);
    jointName = sprintf('joint%d', i);
    bodyNames{i} = bodyName;

    body = rigidBody(bodyName);

    % ---- Create joint based on type ----
    switch jt
        case 'R'
            joint = rigidBodyJoint(jointName, 'revolute');
        case 'P'
            joint = rigidBodyJoint(jointName, 'prismatic');
        case 'F'
            joint = rigidBodyJoint(jointName, 'fixed');
        otherwise
            error('Unknown joint type "%s". Use R, P, or F.', jt);
    end

    % ---- DH transform (your required function) ----
    T = spatialTransform([alpha, a, d, th0]);
    setFixedTransform(joint, T);


    body.Joint = joint;
    addBody(robot, body, parentName);

    parentName = bodyName;
end
end
