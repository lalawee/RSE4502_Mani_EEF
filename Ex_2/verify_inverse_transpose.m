function verify_inverse_transpose(R)
    disp('--- Inverse = Transpose Property ---');
    
    R_inv = inv(R);
    R_trans = R';
    
    disp('R inverse:');
    disp(R_inv);
    
    disp('R transpose:');
    disp(R_trans);
    
    disp('Difference (should be ~0):');
    disp(max(abs(R_inv - R_trans), [], 'all'));
    
    disp('R * R^T (should be Identity):');
    disp(R * R_trans);
    disp(' ');
end