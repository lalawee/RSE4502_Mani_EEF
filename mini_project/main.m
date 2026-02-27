% main.m
% RSE4502 Mini-Project 1: Precision Robotic Welding
% Main script — orchestrates all modules
% Config-agnostic: change configStr to switch robots

clc; clear; close all;

%% --- Configuration ---
configStr  = 'JetArm';   % change to 'RRP' or 'RRR' or 'JetArm' - basically the  jet arm config with an additional  prismatic joint
baseOffset = [1.2, 0.0]; % robot base offset from cylinder center (world frame)

%% --- Build Robot ---
[robot, DH_table, jointTypes, jLimits, vLimits, q_hint] = buildConfig(configStr);
nActive = sum(~strcmpi(jointTypes, 'F'));
fprintf('Active joints: %d\n\n', nActive);


%% --- Initial Configuration ---
target = [0.5 - baseOffset(1); 0; 1.0];   % seam start in robot base frame
[q, init_err, w_init] = findInitialConfig(robot, DH_table, jointTypes, jLimits, target, q_hint);
fprintf('Initial EE error: %.4fm | Manipulability: %.4f\n\n', init_err, w_init);

%% --- Define Welding Path ---
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
singular_hist = false(1, N);
for i = 1:N
    q_hist(:, i) = q;

    % FK — current EE position
    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q(j);
    end
    T_ee = getTransform(robot, q_struct, robot.BodyNames{end});
    ee_hist(:, i) = T_ee(1:3, 4);

    % Module 4: Jacobian
    J = computeJacobian(DH_table, q, jointTypes);

    % Module 5: Jacobian IK with joint limits
    v_des    = path.v_des(4:6, i);
    p_des    = path.pos(:, i);        % desired position at this timestep
    p_curr   = ee_hist(:, i);         % current EE position from FK

    [q, qdot, lambda, w, singular] = jacobianIK(J, q, v_des, dt, jLimits, vLimits, p_curr, p_des, 'k_p', 2.0);   % was 1.0
    singular_hist(i) = singular;
    qdot_hist(:, i) = qdot;
    lambda_hist(i)  = lambda;
    w_hist(i)       = w;
end

fprintf('Done. Max EE Z deviation: %.4f m\n', max(abs(ee_hist(3,:) - 1.0)));
nSingular = sum(singular_hist);
fprintf('Singularity events: %d / %d timesteps\n', nSingular, N);
fprintf('Max joint velocities actually used:\n');
for j = 1:nActive
    fprintf('  Joint %d: %.4f\n', j, max(abs(qdot_hist(j,:))));
end

%% --- Joint Limit Analysis ---
fprintf('Joint limit hits:\n');
anyHit = false;
for j = 1:nActive
    atMin = sum(q_hist(j,:) <= jLimits(j,1) + 1e-6);
    atMax = sum(q_hist(j,:) >= jLimits(j,2) - 1e-6);
    if atMin > 0 || atMax > 0
        fprintf('  Joint %d hit limits: %d at min, %d at max\n', j, atMin, atMax);
        anyHit = true;
    end
end
if ~anyHit
    fprintf('  No joints hit limits.\n');
end



%% --- Save Results ---
save('sim_results.mat', 'q_hist', 'qdot_hist', 'ee_hist', ...
     'w_hist', 'lambda_hist', 'path', 'configStr', 'baseOffset');
fprintf('Results saved to sim_results.mat\n');

%% --- Quick Plot ---
figure('Name', 'Quick Check');
subplot(1,2,1);
plot3(ee_hist(1,:), ee_hist(2,:), ee_hist(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(path.pos(1,:), path.pos(2,:), path.pos(3,:), 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('EE Path vs Desired (%s)', configStr));
legend('Actual', 'Desired');

subplot(1,2,2);
plot(path.t, w_hist, 'b-', 'LineWidth', 1.5); hold on;
yline(0.01, 'r--', 'Threshold');
xlabel('Time (s)'); ylabel('w');
title('Manipulability Index');
grid on;

%% --- Module 6: Visualize ---
visualize(robot, path, q_hist, qdot_hist, ee_hist, w_hist, lambda_hist, baseOffset);