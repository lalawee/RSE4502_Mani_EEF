% main.m
% RSE4502 Mini-Project 1: Precision Robotic Welding
% Main script — orchestrates all modules

clc; clear; close all;

%% --- Module 1: Build Robot ---
[robot, DH_table, jointTypes] = buildConfig('JetArm');

%% --- Module 2: Workspace Analysis ---
ws = analyzeWorkspace(robot, 'nSamples', 50000, 'baseOffset', [0.8, 0.0]);


% Initial joint configuration — set manually to place EE near seam start
% Seam start in robot base frame: [-0.5, 0, 1.0] (at angle 0)
q = zeros(5, 1);
q(1) =  pi;       % base yaw — point toward seam
q(2) =  pi/4;     % shoulder up
q(3) = -pi/4;     % elbow
q(4) =  0;
q(5) =  0;

%% --- Initial Configuration (numerical search using FK only) ---
fprintf('Searching for initial configuration...\n');

target   = [-0.3; 0; 1.0];
best_q   = zeros(6, 1);   % 6x1
best_err = inf;

for q2 = linspace(-pi/2, pi/2, 20)
    for q3 = linspace(-pi, 0, 20)
        for q4 = linspace(-pi/2, pi/2, 10)
            for q5 = 0
                q_try = [1.0; pi; q2; q3; q4; q5];  % 6x1, prismatic=1.0, yaw=pi
                
                q_struct = homeConfiguration(robot);
                for j = 1:6
                    q_struct(j).JointPosition = q_try(j);
                end
                T = getTransform(robot, q_struct, robot.BodyNames{end});
                err = norm(T(1:3,4) - target);
                
                if err < best_err
                    best_err = err;
                    best_q   = q_try;
                end
            end
        end
    end
end

fprintf('Best error: %.4f m\n', best_err);
fprintf('Best q: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', best_q');

q = best_q;


% Refine search around best_q
fprintf('Refining...\n');
bq = best_q;
for q2 = linspace(bq(2)-0.3, bq(2)+0.3, 20)
    for q3 = linspace(bq(3)-0.3, bq(3)+0.3, 20)
        for q4 = linspace(bq(4)-0.3, bq(4)+0.3, 20)
            for q5 = linspace(bq(5)-0.3, bq(5)+0.3, 10)
                q_try = [1.0; bq(2); q2; q3; q4; q5];
                
                q_struct = homeConfiguration(robot);
                for j = 1:6
                    q_struct(j).JointPosition = q_try(j);  % was q(j) — wrong!
                end
                T = getTransform(robot, q_struct, robot.BodyNames{end});
                err = norm(T(1:3,4) - target);
                
                if err < best_err
                    best_err = err;
                    best_q   = q_try;
                end
            end
        end
    end
end

fprintf('Refined error: %.4f m\n', best_err);
fprintf('Refined q: [%.3f, %.3f, %.3f, %.3f, %.3f]\n', best_q');
q = best_q;

%% --- Module 3: Define Welding Path ---
path = defineWeldPath('omega', 0.5, 'dt', 0.05);

%% --- Main Loop ---
fprintf('Running Jacobian IK loop...\n');

N          = path.N;
dt         = path.dt;
q_hist    = zeros(6, N);
qdot_hist = zeros(6, N);
ee_hist    = zeros(3, N);
lambda_hist = zeros(1, N);
w_hist      = zeros(1, N);

for i = 1:N
    % Store current state
    q_hist(:, i) = q;

    % FK — get current EE position
    q_struct = homeConfiguration(robot);
    for j = 1:6
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

    qdot_hist(:, i)  = qdot;
    lambda_hist(i)   = lambda;
    w_hist(i)        = w;
end

fprintf('Done. Max EE Z deviation: %.4f m\n', max(abs(ee_hist(3,:) - 1.0)));

%% --- Quick Plot ---
figure;
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
visualize(robot, path, q_hist, qdot_hist, ee_hist, w_hist, lambda_hist, [0.8, 0.0]);