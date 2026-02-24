%% Exercise 2.3: Inverse Homogeneous Transformation Matrix
clc; clear;

%% ========================================
%          TEST CASE 1.1
%  psi=10°, theta=20°, phi=30°, P=[1,2,3]
% ========================================
disp('========================================');
disp('         TEST CASE 1.1');
disp('  psi=10°, theta=20°, phi=30°, P=[1,2,3]');
disp('========================================');
disp(' ');

psi = 10; theta = 20; phi = 30;
P = [1; 2; 3];

% Build T_BA
R = Rz(psi) * Ry(theta) * Rx(phi);
T_BA = [R, P;
        0 0 0, 1];

disp('Original T_BA:');
disp(T_BA);

% Method 1: Symbolic Formula
% T_inv = | R'    -R'*P |
%         | 0       1   |
R_inv = R';              % Transpose of R
P_inv = -R' * P;         % New translation
T_inv_symbolic = [R_inv, P_inv;
                  0 0 0, 1];

disp('Method 1 - Inverse using SYMBOLIC FORMULA:');
disp(T_inv_symbolic);

% Method 2: MATLAB inv() function
T_inv_matlab = inv(T_BA);

disp('Method 2 - Inverse using MATLAB inv():');
disp(T_inv_matlab);

% Compare the two methods
diff = T_inv_symbolic - T_inv_matlab;
disp('Difference (Symbolic - MATLAB):');
disp(diff);
disp(['Max difference: ', num2str(max(abs(diff), [], 'all'))]);
disp(' ');

% Verify: T_inv * T = Identity
identity_check = T_inv_symbolic * T_BA;
disp('Verification: T_inv * T_BA (should be Identity):');
disp(identity_check);
disp(' ');

%% ========================================
%          TEST CASE 1.2
%  psi=0°, theta=20°, phi=0°, P=[3,0,1]
% ========================================
disp('========================================');
disp('         TEST CASE 1.2');
disp('  psi=0°, theta=20°, phi=0°, P=[3,0,1]');
disp('========================================');
disp(' ');

psi = 0; theta = 20; phi = 0;
P = [3; 0; 1];

% Build T_BA
R = Rz(psi) * Ry(theta) * Rx(phi);
T_BA = [R, P;
        0 0 0, 1];

disp('Original T_BA:');
disp(T_BA);

% Method 1: Symbolic Formula
R_inv = R';
P_inv = -R' * P;
T_inv_symbolic = [R_inv, P_inv;
                  0 0 0, 1];

disp('Method 1 - Inverse using SYMBOLIC FORMULA:');
disp(T_inv_symbolic);

% Method 2: MATLAB inv() function
T_inv_matlab = inv(T_BA);

disp('Method 2 - Inverse using MATLAB inv():');
disp(T_inv_matlab);

% Compare the two methods
diff = T_inv_symbolic - T_inv_matlab;
disp('Difference (Symbolic - MATLAB):');
disp(diff);
disp(['Max difference: ', num2str(max(abs(diff), [], 'all'))]);
disp(' ');

% Verify: T_inv * T = Identity
identity_check1 = T_inv_symbolic * T_BA;
disp('Verification: T_inv * T_BA (should be Identity):');
disp(identity_check1);

% Also verify: T * T_inv = Identity
identity_check2 = T_BA * T_inv_symbolic;
disp('Verification: T_BA * T_inv (should be Identity):');
disp(identity_check2);