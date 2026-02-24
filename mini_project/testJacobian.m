[robot, DH_table, jointTypes] = buildConfig('JetArm');
q_home = homeConfiguration(robot);

% Extract q vector (active joints only)
q_vec = zeros(5,1);
for i = 1:5
    q_vec(i) = q_home(i).JointPosition;
end

% Our Jacobian
J_ours = computeJacobian(DH_table, q_vec, jointTypes);

% MATLAB's Jacobian
J_matlab = geometricJacobian(robot, q_home, robot.BodyNames{end});
J_ours_reordered = [J_ours(4:6,:); J_ours(1:3,:)];
disp('Difference:');
disp(J_ours_reordered - J_matlab)
disp('Our J:');    disp(J_ours)
disp('MATLAB J:'); disp(J_matlab)


disp('MATLAB J columns (linear rows 4-6):');
for i = 1:5
    fprintf('Column %d linear: [%.4f, %.4f, %.4f]\n', i, J_matlab(4,i), J_matlab(5,i), J_matlab(6,i));
end
disp('MATLAB J columns (angular rows 1-3):');
for i = 1:5
    fprintf('Column %d angular: [%.4f, %.4f, %.4f]\n', i, J_matlab(1,i), J_matlab(2,i), J_matlab(3,i));
end