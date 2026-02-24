% Z-Y-X Convention (psi, theta, phi)
clc; clear;

R_B_A = eulerZYX_to_R(0, 20, 0);
disp('Rotation Matrix R_B_A:');
disp(R_B_A);

BP = [1; 0; 1];

disp('A_P');
AP = R_B_A * BP;
disp(AP);







