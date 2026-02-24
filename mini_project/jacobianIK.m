function [q_new, q_dot, lambda, w] = jacobianIK(J, q, v_des, dt, varargin)
% jacobianIK
% Computes joint velocities via Jacobian pseudoinverse (DLS) and integrates
% to get new joint angles. Uses linear velocity rows only (position tracking).
%
% Inputs:
%   J      - 6x5 Jacobian [angular; linear] from computeJacobian
%   q      - 5x1 current joint angles (rad)
%   v_des  - 3x1 desired linear velocity [xdot; ydot; zdot]
%   dt     - timestep (s)
%
% Optional Name-Value:
%   'lambda_max' - max damping coefficient, default 0.1
%   'epsilon'    - manipulability threshold, default 0.01
%
% Outputs:
%   q_new  - 5x1 updated joint angles
%   q_dot  - 5x1 joint velocities
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
J_lin = J(4:6, :);   % 3x5

% --- Manipulability index ---
% w = sqrt(det(J_lin * J_lin'))
w = sqrt(abs(det(J_lin * J_lin')));

% --- Damping coefficient ---
if w < epsilon
    lambda = lambda_max * (1 - (w / epsilon)^2);
else
    lambda = 0;
end

% --- DLS pseudoinverse ---
% q_dot = J_lin' * (J_lin * J_lin' + lambda^2 * I)^{-1} * v_des
J_dls  = J_lin' * ((J_lin * J_lin' + lambda^2 * eye(3)) \ v_des);
q_dot  = J_dls;

% --- Integrate ---
q_new = q + q_dot * dt;

end