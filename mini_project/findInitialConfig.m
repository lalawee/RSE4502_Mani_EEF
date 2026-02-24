function [q_init, final_err, w_init] = findInitialConfig(robot, DH_table, jointTypes, jLimits, target)
% findInitialConfig
% Finds a good initial joint configuration that places the EE near the
% target position with high manipulability.
%
% Inputs:
%   robot      - rigidBodyTree object
%   DH_table   - nx4 DH parameter matrix
%   jointTypes - nx1 cell array of joint types
%   jLimits    - mx2 joint limits [min, max]
%   target     - 3x1 target EE position in robot base frame
%
% Outputs:
%   q_init    - mx1 best joint configuration found
%   final_err - position error at q_init (m)
%   w_init    - manipulability index at q_init

rng(42);   % fixed seed — deterministic results

nActive    = size(jLimits, 1);
nSearch    = 20000;
w_min      = 0.05;   % minimum acceptable manipulability

best_q     = zeros(nActive, 1);
best_err   = inf;
best_w     = 0;
best_score = inf;

fprintf('Searching for initial configuration...\n');

%% --- Coarse random search ---
for s = 1:nSearch
    q_try = jLimits(:,1) + (jLimits(:,2) - jLimits(:,1)) .* rand(nActive, 1);

    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q_try(j);
    end
    T   = getTransform(robot, q_struct, robot.BodyNames{end});
    err = norm(T(1:3,4) - target);

    % Always compute manipulability — score balances both
    J     = computeJacobian(DH_table, q_try, jointTypes);
    J_lin = J(4:6, :);
    w     = sqrt(abs(det(J_lin * J_lin')));

    % Combined score: lower err and higher w both improve score
    score = err - 0.3 * w;   % weight manipulability less than position
    if score < best_score
        best_score = score;
        best_q     = q_try;
        best_err   = err;
        best_w     = w;
    end
end

fprintf('Coarse: err=%.4fm, w=%.4f\n', best_err, best_w);

%% --- Refine around best_q ---
for s = 1:nSearch
    q_try = best_q + (rand(nActive,1) - 0.5) .* 0.15;
    q_try = max(q_try, jLimits(:,1));
    q_try = min(q_try, jLimits(:,2));

    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q_try(j);
    end
    T   = getTransform(robot, q_struct, robot.BodyNames{end});
    err = norm(T(1:3,4) - target);

    J     = computeJacobian(DH_table, q_try, jointTypes);
    J_lin = J(4:6, :);
    w     = sqrt(abs(det(J_lin * J_lin')));

    if w > w_min
        score = err / (w + 0.01);
        if score < best_score
            best_score = score;
            best_q     = q_try;
            best_err   = err;
            best_w     = w;
        end
    end
end

q_init    = best_q;
final_err = best_err;
w_init    = best_w;

fmt = ['Refined: err=%.4fm, w=%.4f\nBest q: [' repmat('%.3f, ', 1, nActive)];
fmt = [fmt(1:end-2) ']\n'];
fprintf(fmt, final_err, w_init, q_init');

if final_err > 0.05
    warning('Initial config error is large (%.4fm). IK may drift.', final_err);
end
if w_init < w_min
    warning('Initial manipulability is low (%.4f). Near singular start.', w_init);
end

end