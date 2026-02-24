%% Part 1: Euler Angles to Rotation Matrix
% Z-Y-X Convention (psi, theta, phi)
clc; clear;
%% === TEST CASE 1.1 ===
disp('========================================');
disp('         TEST CASE 1.1');
disp('   psi=10°, theta=20°, phi=30°');
disp('========================================');

R_B_A_case1 = eulerZYX_to_R(10, 20, 30);
disp('Rotation Matrix R_B_A:');
disp(R_B_A_case1);

% Verify six constraints
verify_orthonormal(R_B_A_case1);

% Verify inverse = transpose
verify_inverse_transpose(R_B_A_case1);

%% === TEST CASE 1.2 ===
disp('========================================');
disp('         TEST CASE 1.2');
disp('   psi=30°, theta=90°, phi=-55°');
disp('========================================');

R_B_A_case2 = eulerZYX_to_R(30, 90, -55);
disp('Rotation Matrix R_B_A:');
disp(R_B_A_case2);

% Verify six constraints
verify_orthonormal(R_B_A_case2);

% Verify inverse = transpose
verify_inverse_transpose(R_B_A_case2);

