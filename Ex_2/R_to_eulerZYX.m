function [sol1_deg, sol2_deg] = R_to_eulerZYX(R)
% R_TO_EULERZYX
% Converts a 3x3 rotation matrix to ZYX Euler angles (DEGREES)
% Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
% Output order: [yaw, pitch, roll]

% reference_ZYX_R = [ ...
%     cos(psi)*cos(theta), ...
%     cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), ...
%     cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
% 
%     sin(psi)*cos(theta), ...
%     sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), ...
%     sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
% 
%    -sin(theta), ...
%     cos(theta)*sin(phi), ...
%     cos(theta)*cos(phi)
% ];




    %% 1. Validate input
    if ~isequal(size(R), [3 3])
        error('Input must be a 3x3 rotation matrix.');
    end

    %% 2. Pitch extraction components
    % For ZYX:
    % R(3,1) = -sin(pitch)
    sin_pitch = -R(3,1);

    % cos(pitch) magnitude (used to detect gimbal lock)
    % when gimbal lock, cos(0) gives 0, effectively making cos_pitch_mag 0
    cos_pitch_mag = sqrt(R(1,1)^2 + R(2,1)^2);

    gimbal_threshold = 1e-10;

    %% 3. Gimbal lock check
    if cos_pitch_mag < gimbal_threshold
        % ----- GIMBAL LOCK CASE -----
        % pitch ≈ ±90°
        pitch_1 = atan2(sin_pitch, cos_pitch_mag);

        % yaw and roll become coupled → set roll = 0 by convention
        yaw_1  = atan2(-R(1,2), R(2,2));
        roll_1 = 0;

        % Second solution is identical in gimbal lock
        yaw_2 = yaw_1;
        pitch_2 = pitch_1;
        roll_2 = roll_1;

    else
        % ----- NORMAL CASE -----

        % ================================================================
        % ZYX EULER ANGLE EXTRACTION — TWO VALID SOLUTIONS
        %
        % For the ZYX convention:
        %   R = Rz(yaw) * Ry(pitch) * Rx(roll)
        %
        % The rotation matrix uniquely determines:
        %   sin(pitch) = -R(3,1)
        %
        % However, cos(pitch) is NOT unique:
        %   cos(pitch) = ± sqrt(1 - sin^2(pitch))
        %
        % This sign ambiguity in cos(pitch) gives rise to TWO valid Euler
        % angle solutions that represent the SAME physical orientation.
        %
        % Key trigonometric identities:
        %   sin(theta) = sin(180° - theta)
        %   cos(theta) = -cos(180° - theta)
        %
        % Therefore:
        %   Solution 2 uses the SAME sin(pitch) but the OPPOSITE cos(pitch),
        %   corresponding to:
        %       pitch_2 = 180° - pitch_1
        %
        % When cos(pitch) flips sign, the yaw and roll numerators must also
        % flip sign to preserve the same rotation matrix.
        % ================================================================
        
        % -------- Solution 1 (principal Euler angles) --------
        % Uses +cos(pitch). This is the "standard" solution and matches
        % the original input angles when they lie within the principal range.
        pitch_1 = atan2(sin_pitch, cos_pitch_mag);
        yaw_1   = atan2(R(2,1), R(1,1));
        roll_1  = atan2(R(3,2), R(3,3));
        
        fprintf('YAW PITCH ROLL (1) [rad]: %.6f  %.6f  %.6f\n', yaw_1, pitch_1, roll_1);
        fprintf('YAW PITCH ROLL (1) [deg]: %.3f  %.3f  %.3f\n', ...
                rad2deg(yaw_1), rad2deg(pitch_1), rad2deg(roll_1));
        
        % -------- Solution 2 (alternate Euler angles) --------
        % Uses -cos(pitch), which yields:
        %   pitch_2 = 180° - pitch_1
        %
        % To maintain the same rotation matrix, yaw and roll must be shifted
        % by 180° (sign flip in atan2 arguments).
        pitch_2 = atan2(sin_pitch, -cos_pitch_mag);
        yaw_2   = atan2(-R(2,1), -R(1,1));
        roll_2  = atan2(-R(3,2), -R(3,3));
        
        fprintf('YAW PITCH ROLL (2) [rad]: %.6f  %.6f  %.6f\n', yaw_2, pitch_2, roll_2);
        fprintf('YAW PITCH ROLL (2) [deg]: %.3f  %.3f  %.3f\n', ...
                rad2deg(yaw_2), rad2deg(pitch_2), rad2deg(roll_2));

    end

    %% 4. Convert to degrees and wrap
    sol1_deg = wrapTo180_local(rad2deg([yaw_1, pitch_1, roll_1]));
    sol2_deg = wrapTo180_local(rad2deg([yaw_2, pitch_2, roll_2]));
end

function angles_deg = wrapTo180_local(angles_deg)
% Wrap angles to [-180, 180] degrees
    angles_deg = mod(angles_deg + 180, 360) - 180;
end