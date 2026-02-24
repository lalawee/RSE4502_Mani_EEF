% main.m
% RSE4502 Mini-Project 1: Precision Robotic Welding
% Main script — orchestrates all modules
% Config-agnostic: change configStr to switch robots

clc; clear; close all;

%% --- Configuration ---
configStr  = 'JetArm';   % change to 'RRP', 'RRR', etc.
baseOffset = [0.8, 0.0]; % robot base offset from cylinder center (world frame)

%% --- Module 1: Build Robot ---
[robot, DH_table, jointTypes] = buildConfig(configStr);

% Count active (non-fixed) joints — drives all array sizes
nActive = sum(~strcmpi(jointTypes, 'F'));
fprintf('Active joints: %d\n\n', nActive);

%% --- Module 2: Workspace Analysis ---
ws = analyzeWorkspace(robot, 'nSamples', 50000, 'baseOffset', baseOffset);

%% --- Initial Configuration (numerical search using FK only) ---
fprintf('Searching for initial configuration...\n');

% Seam start in robot base frame: [r*cos(0) - base_x, 0, z_weld]
target   = [0.5 - baseOffset(1); 0; 1.0];
best_q   = zeros(nActive, 1);
best_err = inf;

% Get joint limits from robot
jLimits = zeros(nActive, 2);
idx = 0;
for i = 1:length(robot.Bodies)
    jt = robot.Bodies{i}.Joint.Type;
    if ~strcmp(jt, 'fixed')
        idx = idx + 1;
        jLimits(idx,:) = robot.Bodies{i}.Joint.PositionLimits;
    end
end

% Coarse search — sample random configs
nSearch = 50000;
for s = 1:nSearch
    q_try = zeros(nActive, 1);
    for j = 1:nActive
        q_try(j) = jLimits(j,1) + (jLimits(j,2) - jLimits(j,1)) * rand();
    end

    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q_try(j);
    end
    T = getTransform(robot, q_struct, robot.BodyNames{end});
    err = norm(T(1:3,4) - target);

    if err < best_err
        best_err = err;
        best_q   = q_try;
    end
end

fmt = ['Best q: [' repmat('%.3f, ', 1, nActive)];
fmt = [fmt(1:end-2) ']\n'];
fprintf('Best error: %.4f m\n', best_err);
fprintf(fmt, best_q');

% Refine search around best_q
fprintf('Refining...\n');
for s = 1:nSearch
    q_try = best_q + (rand(nActive,1) - 0.5) .* 0.2;
    % clamp to joint limits
    q_try = max(q_try, jLimits(:,1));
    q_try = min(q_try, jLimits(:,2));

    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q_try(j);
    end
    T = getTransform(robot, q_struct, robot.BodyNames{end});
    err = norm(T(1:3,4) - target);

    if err < best_err
        best_err = err;
        best_q   = q_try;
    end
end

fprintf('Refined error: %.4f m\n', best_err);
fprintf(fmt, best_q');
q = best_q;

%% --- Module 3: Define Welding Path ---
path = defineWeldPath('omega', 0.5, 'dt', 0.05, 'baseOffset', baseOffset(1));

%% --- Main Loop ---
fprintf('Running Jacobian IK loop...\n');

N           = path.N;
dt          = path.dt;
q_hist      = zeros(nActive, N);
qdot_hist   = zeros(nActive, N);
ee_hist     = zeros(3, N);
lambda_hist = zeros(1, N);
w_hist      = zeros(1, N);

for i = 1:N
    % Store current state
    q_hist(:, i) = q;

    % FK — get current EE position
    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q(j);
    end
    T_ee = getTransform(robot, q_struct, robot.BodyNames{end});
    ee_hist(:, i) = T_ee(1:3, 4);

    % Module 4: Compute Jacobian
    J = computeJacobian(DH_table, q, jointTypes);

    % Desired linear velocity at this timestep
    v_des = path.v_des(4:6, i);   % linear part only

    % Module 5: Jacobian IK
    [q, qdot, lambda, w] = jacobianIK(J, q, v_des, dt);

    qdot_hist(:, i) = qdot;
    lambda_hist(i)  = lambda;
    w_hist(i)       = w;
end

fprintf('Done. Max EE Z deviation: %.4f m\n', max(abs(ee_hist(3,:) - 1.0)));

%% --- Quick Plot ---
figure('Name', 'Quick Check');
subplot(1,2,1);
plot3(ee_hist(1,:), ee_hist(2,:), ee_hist(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(path.pos(1,:), path.pos(2,:), path.pos(3,:), 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('EE Path vs Desired');
legend('Actual', 'Desired');

subplot(1,2,2);
plot(path.t, w_hist, 'b-', 'LineWidth', 1.5); hold on;
yline(0.01, 'r--', 'Threshold');
xlabel('Time (s)'); ylabel('w');
title('Manipulability Index');
grid on;

%% --- Module 6: Visualize ---
visualize(robot, path, q_hist, qdot_hist, ee_hist, w_hist, lambda_hist, baseOffset);


%% Save results
save('sim_results.mat', 'q_hist', 'qdot_hist', 'ee_hist', 'w_hist', 'lambda_hist', 'path');