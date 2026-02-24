% testBuildConfig.m
% Verify JetArm geometry and visualize against welding target

clc; clear; close all;

[robot, DH_table, jointTypes] = buildConfig('JetArm');

% Home configuration
q_home = homeConfiguration(robot);
T_home = getTransform(robot, q_home, robot.BodyNames{end});
fprintf('Home EE position: [%.3f, %.3f, %.3f]\n', T_home(1:3,4)');

% Test a config that should reach toward the welding seam
% Robot base at (1.0, 0, 0), target seam at r=0.5 z=1.0
% So EE needs to reach e.g. (-0.5, 0, 1.0) in robot base frame
% i.e. reach forward and up
q_test = homeConfiguration(robot);


T_test = getTransform(robot, q_test, robot.BodyNames{end});
fprintf('Test EE position: [%.3f, %.3f, %.3f]\n', T_test(1:3,4)');

% Visualize
figure('Name', 'JetArm 6x Scaled');
show(robot, q_test);
title('JetArm (6x Scaled) - Test Configuration');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal; hold on;

% Draw cylinder at world origin
[Xc, Yc, Zc] = cylinder(0.5, 50);
Zc = Zc * 1.0;
% Offset cylinder to world origin (robot base is at x=1.0)
surf(Xc - 1.0, Yc, Zc, 'FaceColor', [0.7 0.7 0.7], ...
     'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Draw welding seam (offset by -1.0 in X since robot base is at x=1.0)
theta_c = linspace(0, 2*pi, 100);
plot3(0.5*cos(theta_c) - 1.0, 0.5*sin(theta_c), ones(1,100), ...
      'r-', 'LineWidth', 2);

legend('Robot', 'Cylinder', 'Welding Seam');