function [q_new, q_dot, lambda, w, singular] = jacobianIK(J, q, v_des, dt, jLimits, vLimits, p_current, p_desired, varargin)
% jacobianIK
% Resolves joint velocities from a desired end-effector linear velocity
% using the Damped Least Squares (DLS) pseudoinverse of the linear Jacobian.
% Integrates joint velocities via forward Euler to update joint positions.
% Includes proportional position feedback to correct accumulated drift.
%
% Inputs:
%   J         - 6xm Jacobian [angular (3xm); linear (3xm)] from computeJacobian
%   q         - mx1 current joint values (rad for revolute, m for prismatic)
%   v_des     - 3x1 desired end-effector linear velocity [xdot; ydot; zdot]
%   dt        - timestep (s)
%   jLimits   - mx2 joint position limits [min, max] for each active joint
%   vLimits   - mx1 max joint velocity magnitudes (rad/s or m/s)
%   p_current - 3x1 current EE position from FK (used for drift correction)
%   p_desired - 3x1 desired EE position on path (used for drift correction)
%
% Optional Name-Value:
%   'lambda_max' - maximum DLS damping coefficient (default: 0.1)
%   'epsilon'    - manipulability threshold for singularity detection (default: 0.01)
%   'k_p'        - proportional gain for position drift correction (default: 1.0)
%
% Outputs:
%   q_new    - mx1 updated joint values, clamped to position limits
%   q_dot    - mx1 joint velocities, clamped to velocity limits
%   lambda   - DLS damping coefficient used this timestep (0 if not near singular)
%   w        - manipulability index: w = sqrt(|det(J_lin * J_lin')|)
%   singular - true if w < epsilon (singularity detected this timestep)

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
% Measures distance from singularity. Near zero = near singular.

w = sqrt(abs(det(J_lin * J_lin')));

% --- Singularity detection and adaptive damping ---
% DLS damping activates smoothly as w approaches zero,
% preventing large joint velocities near singular configurations.

singular = w < epsilon;

if singular
    lambda = lambda_max * (1 - (w / epsilon)^2);
else
    lambda = 0;
end



% --- Position drift correction ---
% Pure velocity integration accumulates error over time.
% Adding k_p * (p_desired - p_current) corrects this drift every timestep.

% v_cmd = v_des + k_p * (p_desired - p_current)
e_pos = p_desired - p_current;
v_cmd = v_des + k_p * e_pos;

% --- DLS pseudoinverse ---
% q_dot = J_lin' * (J_lin * J_lin' + lambda^2 * I)^{-1} * v_cmd


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