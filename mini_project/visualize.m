function visualize(robot, path, q_hist, qdot_hist, ee_hist, w_hist, lambda_hist)
% visualize
% Generates all plots and animation for the welding simulation.
%
% Inputs:
%   robot       - rigidBodyTree object
%   path        - struct from defineWeldPath
%   q_hist      - 5xN joint angles over time
%   qdot_hist   - 5xN joint velocities over time
%   ee_hist     - 3xN EE positions over time
%   w_hist      - 1xN manipulability index over time
%   lambda_hist - 1xN damping values over time

t  = path.t;
N  = path.N;

%% --- Figure 1: Joint Angles ---
figure('Name', 'Joint Angles', 'Position', [50 50 1000 600]);
jointLabels = {'q1 Base Yaw', 'q2 Shoulder', 'q3 Elbow', 'q4 Wrist Pitch', 'q5 Wrist Roll'};
for j = 1:5
    subplot(3,2,j);
    plot(t, q_hist(j,:), 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title(jointLabels{j});
    grid on;
end
sgtitle('Joint Angles vs Time', 'FontSize', 14, 'FontWeight', 'bold');

%% --- Figure 2: Joint Velocities ---
figure('Name', 'Joint Velocities', 'Position', [100 50 1000 600]);
for j = 1:5
    subplot(3,2,j);
    plot(t, qdot_hist(j,:), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Vel (rad/s)');
    title(jointLabels{j});
    grid on;
end
sgtitle('Joint Velocities vs Time', 'FontSize', 14, 'FontWeight', 'bold');

%% --- Figure 3: EE Tracking ---
figure('Name', 'EE Tracking', 'Position', [150 50 1000 800]);

% 3D path
subplot(2,2,[1,2]);
plot3(ee_hist(1,:), ee_hist(2,:), ee_hist(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(path.pos(1,:), path.pos(2,:), path.pos(3,:), 'r--', 'LineWidth', 1.5);
% Draw cylinder at world origin (offset -1.0 in X in robot base frame)
[Xc, Yc, Zc] = cylinder(0.5, 50);
Zc = Zc * 1.0;
surf(Xc - 1.0, Yc, Zc, 'FaceColor', [0.7 0.7 0.7], ...
     'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-Effector Path vs Desired Welding Seam');
legend('Actual EE', 'Desired Seam', 'Workpiece');
view(35, 30);

% X tracking
subplot(2,2,3);
plot(t, ee_hist(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, path.pos(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
title('EE X vs Time');
legend('Actual', 'Desired'); grid on;

% Tracking error
subplot(2,2,4);
err = vecnorm(ee_hist - path.pos(:,1:N));
plot(t, err * 1000, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (mm)');
title('EE Position Tracking Error');
grid on;

sgtitle('End-Effector Tracking Performance', 'FontSize', 14, 'FontWeight', 'bold');

%% --- Figure 4: Singularity Analysis ---
figure('Name', 'Singularity Analysis', 'Position', [200 50 900 400]);

subplot(1,2,1);
plot(t, w_hist, 'b-', 'LineWidth', 1.5); hold on;
yline(0.01, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('w');
title('Manipulability Index');
legend('w', 'Threshold \epsilon=0.01');
grid on;

subplot(1,2,2);
plot(t, lambda_hist, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\lambda');
title('DLS Damping Coefficient');
grid on;

sgtitle('Singularity Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% --- Figure 5: Animation ---
figure('Name', '3D Animation', 'Position', [250 50 800 800], 'Color', 'w');
ax = axes;
view(35, 30); hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('JetArm Welding Simulation', 'FontSize', 14);

% Static elements
[Xc, Yc, Zc] = cylinder(0.5, 50);
Zc = Zc * 1.0;
surf(Xc-1.0, Yc, Zc, 'FaceColor', [0.7 0.7 0.7], ...
     'FaceAlpha', 0.4, 'EdgeColor', 'none');
plot3(path.pos(1,:), path.pos(2,:), path.pos(3,:), ...
      'k--', 'LineWidth', 1.5);

% Weld trace
trace = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 3);
tx = []; ty = []; tz = [];

fprintf('Playing animation...\n');
for i = 1:3:N
    if ~isvalid(ax), break; end

    % Build q struct
    q_struct = homeConfiguration(robot);
    for j = 1:5
        q_struct(j).JointPosition = q_hist(j, i);
    end

    show(robot, q_struct, 'Parent', ax, ...
         'PreservePlot', false, 'FastUpdate', true);

    % Update weld trace
    tx = [tx, ee_hist(1,i)];
    ty = [ty, ee_hist(2,i)];
    tz = [tz, ee_hist(3,i)];
    set(trace, 'XData', tx, 'YData', ty, 'ZData', tz);

    drawnow;
    pause(0.02);
end

fprintf('Animation complete.\n');
end