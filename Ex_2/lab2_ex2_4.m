%% Exercise 2.4: Chaining Transformations
% Find C_T_A from B_T_A and C_T_B
clc; clear;

%% ========================================
%  Step 1: Define B_T_A (from Ex2 Case 1.1)
% ========================================
disp('========================================');
disp('  Step 1: B_T_A (from Case 1.1)');
disp('  psi=10°, theta=20°, phi=30°, P=[1,2,3]');
disp('========================================');

psi = 10; theta = 20; phi = 30;
P_BA = [1; 2; 3];

R_BA = Rz(psi) * Ry(theta) * Rx(phi);
B_T_A = [R_BA, P_BA;
         0 0 0, 1];

disp('B_T_A:');
disp(B_T_A);

%% ========================================
%  Step 2: Define C_T_B (from Ex2 Case 1.2)
% ========================================
disp('========================================');
disp('  Step 2: C_T_B (from Case 1.2)');
disp('  psi=0°, theta=20°, phi=0°, P=[3,0,1]');
disp('========================================');

psi = 0; theta = 20; phi = 0;
P_CB = [3; 0; 1];

R_CB = Rz(psi) * Ry(theta) * Rx(phi);
C_T_B = [R_CB, P_CB;
         0 0 0, 1];

disp('C_T_B:');
disp(C_T_B);

%% ========================================
%  Step 3: Calculate C_T_A
% ========================================
disp('========================================');
disp('  Step 3: Calculate C_T_A');
disp('  C_T_A = C_T_B * B_T_A');
disp('========================================');

C_T_A = C_T_B * B_T_A;

disp('C_T_A:');
disp(C_T_A);

%% ========================================
%  Step 4: Transform Graph
% ========================================
disp('========================================');
disp('  TRANSFORM GRAPH');
disp('========================================');
disp(' ');
disp('        B_T_A           C_T_B');
disp('   A ──────────► B ──────────► C');
disp('   │                           ▲');
disp('   │                           │');
disp('   └───────────────────────────┘');
disp('              C_T_A');
disp(' ');
disp('Equation: C_T_A = C_T_B * B_T_A');
disp(' ');

%% ========================================
%  Step 5: Visualize All Three Frames
% ========================================
% Frame A: at origin (Identity)
% Frame B: relative to A
% Frame C: relative to A (using C_T_A)

figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Exercise 2.4: Three Frames A, B, C');

% Plot Frame A (World Frame at origin)
plotFrame(eye(4), 'A', 'k');

% Plot Frame B (relative to A)
plotFrame(B_T_A, 'B', 'b');

% Plot Frame C (relative to A)
plotFrame(C_T_A, 'C', 'r');

% Adjust view
view(30, 20);
xlim([-1 6]);
ylim([-1 5]);
zlim([-1 6]);
legend('Location', 'bestoutside');

hold off;

