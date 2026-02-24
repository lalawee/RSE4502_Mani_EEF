%% Exercise 2.2: Transform Mapping
% Given: psi=0°, theta=20°, phi=0°, P_org=[3,0,1], P1_B=[1,0,1]
% Find: P1_A
clc; clear;

%% Step 1: Build Homogeneous Transformation Matrix (from Ex2.1 Case 1.2)
disp('========================================');
disp('         EXERCISE 2.2');
disp('========================================');
disp(' ');

psi = 0; theta = 20; phi = 0;
P_org = [3; 0; 1];  % Position of Frame B origin relative to Frame A

R_BA = Rz(psi) * Ry(theta) * Rx(phi);
T_BA = [R_BA, P_org;
        0 0 0, 1];

disp('Transformation Matrix T_BA:');
disp(T_BA);

%% Step 2: Define Point in Frame B
P1_B = [1; 0; 1];
disp('Point in Frame B:');
disp(['  P1_B = [', num2str(P1_B'), ']']);

%% Step 3: Convert to Homogeneous Coordinates
P1_B_hom = [P1_B; 1];  % Add 1 at the bottom
disp('Point in homogeneous coordinates:');
disp(['  P1_B_hom = [', num2str(P1_B_hom'), ']']);

%% Step 4: Transform Point from Frame B to Frame A
P1_A_hom = T_BA * P1_B_hom;
disp('Transformed point (homogeneous):');
disp(['  P1_A_hom = [', num2str(P1_A_hom'), ']']);

% Extract the 3D coordinates (remove the 1 at bottom)
P1_A = P1_A_hom(1:3);
disp('Point in Frame A:');
disp(['  P1_A = [', num2str(P1_A'), ']']);

%% Step 5: Visualize
visualizeFrames(T_BA);
hold on;

% Plot P1_B in Frame B coordinates (cyan)
% To show where P1_B is in the world, we need to transform it
plot3(P1_A(1), P1_A(2), P1_A(3), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2);
text(P1_A(1)+0.1, P1_A(2)+0.1, P1_A(3)+0.1, ['P1_A = [', num2str(round(P1_A',2)), ']'], 'FontSize', 10, 'Color', 'm');

% Show the original point location label
text(P_org(1)+0.2, P_org(2), P_org(3)+0.2, 'Frame B', 'FontSize', 10, 'Color', 'k');

% Adjust view
view(30, 20);
xlim([-1 5]);
ylim([-1 3]);
zlim([-1 3]);
title('Exercise 2.2: Transform Mapping (P1_B → P1_A)');

hold off;

%% ========================================
%  THREE INTERPRETATIONS OF T_BA
% ========================================
disp(' ');
disp('========================================');
disp('  THREE INTERPRETATIONS OF T_BA');
disp('========================================');
disp(' ');

disp('1. FRAME DESCRIPTION:');
disp('   T_BA describes Frame B relative to Frame A');
disp(['   - Frame B is rotated ', num2str(theta), '° about Y-axis']);
disp(['   - Frame B origin is at [', num2str(P_org'), '] in Frame A']);
disp(' ');

disp('2. TRANSFORM MAPPING (This exercise!):');
disp('   T_BA maps points from Frame B coords to Frame A coords');
disp(['   - P1_B = [', num2str(P1_B'), '] in Frame B']);
disp(['   - P1_A = [', num2str(round(P1_A',4)), '] in Frame A']);
disp('   - Formula: [P1_A;1] = T_BA * [P1_B;1]');
disp(' ');

disp('3. OPERATOR (Rigid motion acting on a vector):');
disp('   Applying T_BA performs: rotate by R_BA, then translate by P_org');
disp('   - Equivalent: P1_A = R_BA*P1_B + P_org');
disp(' ');





