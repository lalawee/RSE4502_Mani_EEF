function [q_new, q_dot, lambda, w] = jacobianIK(J, q, v_des, dt, jLimits, varargin)
% jacobianIK
% Computes joint velocities via Jacobian pseudoinverse (DLS) and integrates
% to get new joint angles. Uses linear velocity rows only (position tracking).
%
% Inputs:
%   J       - 6xn Jacobian [angular; linear] from computeJacobian
%   q       - nx1 current joint values (rad or m)
%   v_des   - 3x1 desired linear velocity [xdot; ydot; zdot]
%   dt      - timestep (s)
%   jLimits - nx2 joint limits [min, max] for each joint
%
% Optional Name-Value:
%   'lambda_max' - max damping coefficient, default 0.1
%   'epsilon'    - manipulability threshold, default 0.01
%
% Outputs:
%   q_new  - nx1 updated joint values (clamped to limits)
%   q_dot  - nx1 joint velocities
%   lambda - damping value used this step
%   w      - manipulability index

% --- Parameters ---
p = inputParser;
addParameter(p, 'lambda_max', 0.1);
addParameter(p, 'epsilon',    0.01);
parse(p, varargin{:});

lambda_max = p.Results.lambda_max;
epsilon    = p.Results.epsilon;

% --- Extract linear velocity rows only ---
J_lin = J(4:6, :);   % 3xn

% --- Manipulability index ---
w = sqrt(abs(det(J_lin * J_lin')));

% --- Damping coefficient ---
if w < epsilon
    lambda = lambda_max * (1 - (w / epsilon)^2);
else
    lambda = 0;
end

% --- DLS pseudoinverse ---
q_dot = J_lin' * ((J_lin * J_lin' + lambda^2 * eye(3)) \ v_des);

% --- Integrate ---
q_new = q + q_dot * dt;

% --- Clamp to joint limits ---
q_new = max(q_new, jLimits(:,1));
q_new = min(q_new, jLimits(:,2));

end