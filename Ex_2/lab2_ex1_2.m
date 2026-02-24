%% --- REFINED TEST SUITE ---
clear; clc;

% 1. Define your test angles (In DEGREES)
original_yaw   = 10; 
original_pitch = 20; 
original_roll  = 30;

% 2. Generate the "Ground Truth" Matrix using your function
R_ground_truth = eulerZYX_to_R(original_yaw, original_pitch, original_roll);
fprintf('=== TEST: ZYX Extraction ===\n');
fprintf('Input Angles: [%g, %g, %g] degrees\n\n', original_yaw, original_pitch, original_roll);
disp('R_ground_truth =')
disp(R_ground_truth)
% 3. Extract the two solutions
[sol1, sol2] = R_to_eulerZYX(R_ground_truth);

% 4. Validate results
fprintf('Recovered Sol 1: [%.2f, %.2f, %.2f] deg\n', sol1);
fprintf('Recovered Sol 2: [%.2f, %.2f, %.2f] deg\n', sol2);

% 5. Accuracy Check (Reconstruction)
% Check if Sol 2 actually produces the same matrix as Sol 1
R_check2 = eulerZYX_to_R(sol2(1), sol2(2), sol2(3));
reconstruction_error = norm(R_ground_truth - R_check2, 'fro');

fprintf('\nMatrix Reconstruction Error (Sol 2): %.2e\n', reconstruction_error);

if reconstruction_error < 1e-12
    fprintf('VERIFIED: Both solutions represent the same 3D orientation.\n');
end