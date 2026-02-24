function R = eulerZYX_to_R(psi_deg, theta_deg, phi_deg)
    % Converts Z-Y-X Euler angles (in degrees) to rotation matrix
    % Combined: Z-Y-X order
    R = Rz(psi_deg) * Ry(theta_deg) * Rx(phi_deg);
end