%% Exercise 2.1: Homogeneous Transformation Matrix
clc; clear;

%% ===== Test Case 1.1 =====
disp('========================================');
disp('         TEST CASE 1.1');
disp('  psi=10°, theta=20°, phi=30°');
disp('  P_org = [1, 2, 3]');
disp('========================================');

psi = 10; theta = 20; phi = 30;
P_BA = [1; 2; 3];

R_BA = Rz(psi) * Ry(theta) * Rx(phi);
T_BA_case1 = [R_BA, P_BA;
              0 0 0, 1];

disp('Homogeneous Transformation Matrix T_BA:');
disp(T_BA_case1);

% Visualize (optional)
visualizeFrames(T_BA_case1);

%% ===== Test Case 1.2 =====
disp('========================================');
disp('         TEST CASE 1.2');
disp('  psi=0°, theta=20°, phi=0°');
disp('  P_org = [3, 0, 1]');
disp('========================================');

psi = 0; theta = 20; phi = 0;
P_BA = [3; 0; 1];

R_BA = Rz(psi) * Ry(theta) * Rx(phi);
T_BA_case2 = [R_BA, P_BA;
              0 0 0, 1];

disp('Homogeneous Transformation Matrix T_BA:');
disp(T_BA_case2);

% Visualize (optional)
visualizeFrames(T_BA_case2);
