%% Exercise 2.5: Loop Equation
% Given C_T_A and C_T_B, find B_T_A
clc; clear;

%% ========================================
%  Step 1: Rebuild C_T_B and C_T_A from Q4
% ========================================
disp('========================================');
disp('  Step 1: Known Matrices (from Q4)');
disp('========================================');
disp(' ');

% B_T_A (from Case 1.1) - we'll use this to verify later
psi = 10; theta = 20; phi = 30;
P_BA = [1; 2; 3];
R_BA = Rz(psi) * Ry(theta) * Rx(phi);
B_T_A_original = [R_BA, P_BA;
                  0 0 0, 1];

% C_T_B (from Case 1.2)
psi = 0; theta = 20; phi = 0;
P_CB = [3; 0; 1];
R_CB = Rz(psi) * Ry(theta) * Rx(phi);
C_T_B = [R_CB, P_CB;
         0 0 0, 1];

% C_T_A (calculated in Q4)
C_T_A = C_T_B * B_T_A_original;

disp('C_T_A (known from Q4):');
disp(C_T_A);

disp('C_T_B (known from Q4):');
disp(C_T_B);

%% ========================================
%  Step 2: Calculate B_T_A using Loop Equation
% ========================================
disp('========================================');
disp('  Step 2: Solve for B_T_A');
disp('========================================');
disp(' ');

disp('Loop Equation:');
disp('  C_T_A = C_T_B * B_T_A');
disp(' ');
disp('Rearranged:');
disp('  B_T_A = inv(C_T_B) * C_T_A');
disp('  B_T_A = B_T_C * C_T_A');
disp(' ');

% Method 1: Using inv()
B_T_A_calculated = inv(C_T_B) * C_T_A;

disp('B_T_A (calculated using loop equation):');
disp(B_T_A_calculated);

%% ========================================
%  Step 3: Compare with Original B_T_A
% ========================================
disp('========================================');
disp('  Step 3: Verification');
disp('========================================');
disp(' ');

disp('B_T_A (original from Case 1.1):');
disp(B_T_A_original);

disp('B_T_A (calculated from loop equation):');
disp(B_T_A_calculated);

% Calculate difference
difference = B_T_A_original - B_T_A_calculated;
disp('Difference (Original - Calculated):');
disp(difference);

max_error = max(abs(difference), [], 'all');
disp(['Maximum error: ', num2str(max_error)]);
disp(' ');

if max_error < 1e-10
    disp('✓ SUCCESS! Loop equation gives the correct B_T_A');
else
    disp('✗ ERROR! Something went wrong');
end

%% ========================================
%  Step 4: Show the Loop Equation Graphically
% ========================================
disp(' ');
disp('========================================');
disp('  TRANSFORM GRAPH & LOOP EQUATION');
disp('========================================');
disp(' ');
disp('        B_T_A           C_T_B');
disp('   A ──────────► B ──────────► C');
disp('   │             ▲             │');
disp('   │             │             │');
disp('   │             └─────────────┘');
disp('   │                B_T_C = inv(C_T_B)');
disp('   │                           ');
disp('   └───────────────────────────┘');
disp('              C_T_A');
disp(' ');
disp('Loop Equation:');
disp('  C_T_A = C_T_B * B_T_A');
disp(' ');
disp('Solving for unknown B_T_A:');
disp('  B_T_A = inv(C_T_B) * C_T_A');
disp('  B_T_A = B_T_C * C_T_A');
disp(' ');

%% ===== Helper Functions =====
function R = Rz(psi)
    R = [cosd(psi) -sind(psi) 0;
         sind(psi)  cosd(psi) 0;
         0          0         1];
end

function R = Ry(theta)
    R = [cosd(theta)  0  sind(theta);
         0            1  0;
        -sind(theta)  0  cosd(theta)];
end

function R = Rx(phi)
    R = [1  0          0;
         0  cosd(phi) -sind(phi);
         0  sind(phi)  cosd(phi)];
end