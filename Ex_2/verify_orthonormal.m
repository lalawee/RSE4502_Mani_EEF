function verify_orthonormal(R)
    disp('--- Six Constraints Verification ---');
    
    col1 = R(:,1);
    col2 = R(:,2);
    col3 = R(:,3);
    
    % Constraint 1-3: Unit vectors
    disp('Column magnitudes (should = 1):');
    disp(['  |col1| = ', num2str(norm(col1))]);
    disp(['  |col2| = ', num2str(norm(col2))]);
    disp(['  |col3| = ', num2str(norm(col3))]);
    
    % Constraint 4-6: Perpendicular
    disp('Dot products (should = 0):');
    disp(['  col1·col2 = ', num2str(dot(col1,col2))]);
    disp(['  col1·col3 = ', num2str(dot(col1,col3))]);
    disp(['  col2·col3 = ', num2str(dot(col2,col3))]);
    disp(' ');
end