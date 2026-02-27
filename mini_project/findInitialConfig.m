function [q_init, final_err, w_init] = findInitialConfig(robot, DH_table, jointTypes, jLimits, target, q_hint)
% findInitialConfig
% Finds a good initial joint configuration using MATLAB's numerical IK solver.
% Config-agnostic — works for any rigidBodyTree robot.
%
% Inputs:
%   robot      - rigidBodyTree object
%   DH_table   - nx4 DH parameter matrix
%   jointTypes - nx1 cell array of joint types
%   jLimits    - mx2 joint limits [min, max]
%   target     - 3x1 target EE position in robot base frame
%   q_hint     - mx1 initial guess for IK solver (optional)
%
% Outputs:
%   q_init    - mx1 best joint configuration found
%   final_err - position error at q_init (m)
%   w_init    - manipulability index at q_init

nActive = size(jLimits, 1);

% Default hint: midpoint of joint limits
if nargin < 6 || isempty(q_hint)
    q_hint = mean(jLimits, 2);
end

fprintf('Solving for initial configuration (MATLAB IK)...\n');

%% --- Setup IK solver ---
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations  = 1500;
ik.SolverParameters.MaxTime        = 10;
ik.SolverParameters.GradientTolerance = 1e-8;
ik.SolverParameters.SolutionTolerance = 1e-6;

% Position only — ignore orientation (weights: [orientation(3), position(3)])
weights = [0 0 0 1 1 1];

% Target as 4x4 homogeneous transform (position only, identity orientation)
T_target = eye(4);
T_target(1:3, 4) = target;

% Initial guess struct
q0_struct = homeConfiguration(robot);
for j = 1:nActive
    q0_struct(j).JointPosition = q_hint(j);
end

%% --- Solve ---
[configSoln, solnInfo] = ik(robot.BodyNames{end}, T_target, weights, q0_struct);

q_init = zeros(nActive, 1);
for j = 1:nActive
    q_init(j) = configSoln(j).JointPosition;
end

%% --- Verify solution ---
q_struct = homeConfiguration(robot);
for j = 1:nActive
    q_struct(j).JointPosition = q_init(j);
end
T_ee      = getTransform(robot, q_struct, robot.BodyNames{end});
final_err = norm(T_ee(1:3,4) - target);

%% --- Manipulability at solution ---
J     = computeJacobian(DH_table, q_init, jointTypes);
J_lin = J(4:6, :);
w_init = sqrt(abs(det(J_lin * J_lin')));

fmt = ['Solution: err=%.4fm, w=%.4f\nBest q: [' repmat('%.3f, ', 1, nActive)];
fmt = [fmt(1:end-2) ']\n'];
fprintf(fmt, final_err, w_init, q_init');
fprintf('IK status: %s\n', solnInfo.Status);

if final_err > 0.05
    warning('Initial config error is large (%.4fm). IK may drift.', final_err);
end
if w_init < 0.05
    warning('Initial manipulability is low (%.4f). Near singular start.', w_init);
end

end