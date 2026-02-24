function [q_new, q_dot, lambda, w, singular] = jacobianIK(J, q, v_des, dt, jLimits, vLimits, p_current, p_desired, varargin)
% jacobianIK
% Computes joint velocities via Jacobian pseudoinverse (DLS) and integrates
% to get new joint angles. Uses linear velocity rows only (position tracking).
%
% Inputs:
%   J         - 6xn Jacobian [angular; linear] from computeJacobian
%   q         - nx1 current joint values (rad or m)
%   v_des     - 3x1 desired linear velocity [xdot; ydot; zdot]
%   dt        - timestep (s)
%   jLimits   - nx2 joint limits [min, max] for each joint
%   vLimits   - nx1 max joint velocity magnitudes (rad/s or m/s)
%   p_current - 3x1 current EE position (from FK)
%   p_desired - 3x1 desired EE position on path
%
% Optional Name-Value:
%   'lambda_max' - max damping coefficient, default 0.1
%   'epsilon'    - manipulability threshold, default 0.01
%   'k_p'        - position feedback gain, default 1.0
%
% Outputs:
%   q_new    - nx1 updated joint values (clamped to position limits)
%   q_dot    - nx1 joint velocities (clamped to velocity limits)
%   lambda   - damping value used this step
%   w        - manipulability index
%   singular - true if singularity was detected this step

% --- Parameters ---
p = inputParser;
addParameter(p, 'lambda_max', 0.1);
addParameter(p, 'epsilon',    0.01);
addParameter(p, 'k_p',        1.0);
parse(p, varargin{:});

lambda_max = p.Results.lambda_max;
epsilon    = p.Results.epsilon;
k_p        = p.Results.k_p;

% --- Extract linear velocity rows only ---
J_lin = J(4:6, :);   % 3xn

% --- Manipulability index ---
w = sqrt(abs(det(J_lin * J_lin')));

% --- Singularity detection ---
singular = w < epsilon;

if singular
    lambda = lambda_max * (1 - (w / epsilon)^2);
else
    lambda = 0;
end

% --- Position error feedback ---
% Corrects accumulated drift by adding a proportional feedback term
% v_cmd = v_des + k_p * (p_desired - p_current)
e_pos = p_desired - p_current;
v_cmd = v_des + k_p * e_pos;

% --- DLS pseudoinverse with feedback-corrected velocity ---
q_dot = J_lin' * ((J_lin * J_lin' + lambda^2 * eye(3)) \ v_cmd);

% --- Clamp joint velocities to velocity limits ---
q_dot = max(q_dot, -vLimits);
q_dot = min(q_dot,  vLimits);

% --- Integrate ---
q_new = q + q_dot * dt;

% --- Clamp to joint position limits ---
q_new = max(q_new, jLimits(:,1));
q_new = min(q_new, jLimits(:,2));

end