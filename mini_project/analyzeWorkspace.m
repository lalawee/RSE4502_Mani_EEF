function ws = analyzeWorkspace(robot, varargin)
% analyzeWorkspace
% Monte Carlo workspace analysis for any rigidBodyTree robot.
%
% Inputs:
%   robot       - rigidBodyTree object
%   Optional Name-Value:
%     'nSamples'  - number of random samples, default 50000
%     'basOffset' - [x,y] robot base offset from world origin, default [1,0]
%
% Output: ws struct with fields
%   points    - 3xN reachable EE positions (robot base frame)
%   points_w  - 3xN reachable EE positions (world frame)
%   r_reach   - radial distance from world origin for each point

% --- Parameters ---
p = inputParser;
addParameter(p, 'nSamples',  50000);
addParameter(p, 'baseOffset', [1.0, 0.0]);
parse(p, varargin{:});

nSamples   = p.Results.nSamples;
baseOffset = p.Results.baseOffset;

% --- Get joint limits ---
activeJoints = {};
for i = 1:length(robot.Bodies)
    jt = robot.Bodies{i}.Joint.Type;
    if ~strcmp(jt, 'fixed')
        activeJoints{end+1} = robot.Bodies{i}.Joint;
    end
end
nJoints = length(activeJoints);

% Extract limits
jLimits = zeros(nJoints, 2);
for i = 1:nJoints
    jLimits(i,:) = activeJoints{i}.PositionLimits;
end

fprintf('Workspace Analysis: %d samples, %d active joints\n', nSamples, nJoints);

% --- Monte Carlo sampling ---
ws_points = zeros(3, nSamples);
endBody   = robot.BodyNames{end};

for s = 1:nSamples
    % Random joint config within limits
    q_struct = homeConfiguration(robot);
    for j = 1:nJoints
        q_struct(j).JointPosition = jLimits(j,1) + ...
            (jLimits(j,2) - jLimits(j,1)) * rand();
    end

    T = getTransform(robot, q_struct, endBody);
    ws_points(:, s) = T(1:3, 4);
end

% --- Convert to world frame ---
ws_world = ws_points;
ws_world(1,:) = ws_points(1,:) + baseOffset(1);
ws_world(2,:) = ws_points(2,:) + baseOffset(2);

% Radial distance from world origin (cylinder axis)
r_reach = sqrt(ws_world(1,:).^2 + ws_world(2,:).^2);

% --- Welding seam in robot base frame ---
theta_c  = linspace(0, 2*pi, 200);
seam_x   = 0.5*cos(theta_c) - baseOffset(1);
seam_y   = 0.5*sin(theta_c);
seam_z   = ones(1, 200);

% Seam in world frame
seam_wx  = seam_x + baseOffset(1);
seam_wy  = seam_y;

% --- Check seam reachability ---
seamPts = [seam_x; seam_y; seam_z];
nReach  = 0;
thresh  = 0.05;  % 5cm tolerance
for k = 1:size(seamPts, 2)
    dists = vecnorm(ws_points - seamPts(:,k));
    if min(dists) < thresh
        nReach = nReach + 1;
    end
end
pctReach = 100 * nReach / size(seamPts, 2);

fprintf('Seam reachability: %.1f%% of seam points within %.0fcm of sampled workspace\n', ...
        pctReach, thresh*100);
fprintf('Workspace bounds (robot base frame):\n');
fprintf('  X: [%.3f, %.3f] m\n', min(ws_points(1,:)), max(ws_points(1,:)));
fprintf('  Y: [%.3f, %.3f] m\n', min(ws_points(2,:)), max(ws_points(2,:)));
fprintf('  Z: [%.3f, %.3f] m\n', min(ws_points(3,:)), max(ws_points(3,:)));

%% --- Plot 1: 3D Workspace Cloud ---
figure('Name', 'Workspace Analysis 3D', 'Position', [50 50 900 700]);

% Color by height
scatter3(ws_points(1,:), ws_points(2,:), ws_points(3,:), ...
         1, ws_points(3,:), 'filled');
colormap('jet'); colorbar;
hold on;

% Draw welding seam
plot3(seam_x, seam_y, seam_z, 'r-', 'LineWidth', 3);
plot3(seam_x(1), seam_y(1), seam_z(1), 'r*', 'MarkerSize', 15);

% Draw cylinder (in robot base frame)
[Xc, Yc, Zc] = cylinder(0.5, 50);
Zc = Zc * 1.0;
surf(Xc - baseOffset(1), Yc, Zc, ...
     'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('3D Reachable Workspace (%d samples)', nSamples));
legend('Reachable Points', 'Welding Seam', 'Seam Start', 'Workpiece');
grid on; axis equal; view(35, 30);

%% --- Plot 2: Cross Section (r vs z in world frame) ---
figure('Name', 'Workspace Cross Section', 'Position', [100 100 700 500]);

scatter(r_reach, ws_world(3,:), 1, 'b', 'filled');
hold on;

% Mark welding seam target
plot(0.5, 1.0, 'r*', 'MarkerSize', 20, 'LineWidth', 2);

% Mark seam range
theta_check = linspace(0, 2*pi, 200);
seam_r = sqrt((0.5*cos(theta_check)).^2 + (0.5*sin(theta_check)).^2);
plot(seam_r, ones(1,200), 'r-', 'LineWidth', 2);

xlabel('Radial Distance from World Origin r (m)');
ylabel('Height z (m)');
title('Workspace Cross-Section (r vs z, World Frame)');
legend('Reachable Points', 'Welding Target (r=0.5, z=1.0)', 'Welding Seam');
grid on;

%% --- Plot 3: XY top-down view ---
figure('Name', 'Workspace Top View', 'Position', [150 150 700 600]);

scatter(ws_world(1,:), ws_world(2,:), 1, ws_world(3,:), 'filled');
colormap('jet'); colorbar;
hold on;

% Draw cylinder
theta_cyl = linspace(0, 2*pi, 100);
fill(0.5*cos(theta_cyl), 0.5*sin(theta_cyl), [0.7 0.7 0.7], 'FaceAlpha', 0.5);
plot(seam_wx, seam_wy, 'r-', 'LineWidth', 2);

% Mark robot base
plot(baseOffset(1), baseOffset(2), 'k^', 'MarkerSize', 15, ...
     'MarkerFaceColor', 'k');

xlabel('X (m)'); ylabel('Y (m)');
title('Workspace Top View (World Frame, colored by Z height)');
legend('Reachable Points', 'Cylinder', 'Welding Seam', 'Robot Base');
axis equal; grid on;

% --- Pack output ---
ws.points   = ws_points;
ws.points_w = ws_world;
ws.r_reach  = r_reach;
ws.pctReach = pctReach;

end
