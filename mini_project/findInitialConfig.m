function [q_init, final_err, w_init] = findInitialConfig(robot, DH_table, jointTypes, jLimits, target, q_hint)
% findInitialConfig
% Finds a good initial joint configuration using Top-K selection.
%
% Phase 1 — Mixed random + hint-centered search, collect top-K by position error
% Phase 2 — Among top-K, select highest manipulability
%
% Inputs:
%   robot      - rigidBodyTree object
%   DH_table   - nx4 DH parameter matrix
%   jointTypes - nx1 cell array of joint types
%   jLimits    - mx2 joint limits [min, max]
%   target     - 3x1 target EE position in robot base frame
%   q_hint     - mx1 rough posture hint (optional, pass [] to disable)
%
% Outputs:
%   q_init    - mx1 best joint configuration found
%   final_err - position error at q_init (m)
%   w_init    - manipulability index at q_init

nActive    = size(jLimits, 1);
nSearch    = 50000;
K          = 100;
err_thresh = 0.08;

% Default hint: midpoint of joint limits
if nargin < 6 || isempty(q_hint)
    q_hint = mean(jLimits, 2);
end

fprintf('Searching for initial configuration (Top-K, N=%d, K=%d)...\n', nSearch, K);

% Top-K storage
topK_q   = zeros(nActive, K);
topK_err = inf(1, K);
[~, worst_idx] = max(topK_err);

%% --- Phase 1: Mixed search ---
for s = 1:nSearch
    if mod(s, 10) <= 3   % 30% random, 70% hint-centered
        q_try = jLimits(:,1) + (jLimits(:,2) - jLimits(:,1)) .* rand(nActive,1);
    else
        % 50% hint-centered with decreasing spread
        spread = 1.5 * (1 - s/nSearch) + 0.3;
        q_try  = q_hint + (rand(nActive,1) - 0.5) .* spread;
        q_try  = max(q_try, jLimits(:,1));
        q_try  = min(q_try, jLimits(:,2));
    end

    q_struct = homeConfiguration(robot);
    for j = 1:nActive
        q_struct(j).JointPosition = q_try(j);
    end
    T   = getTransform(robot, q_struct, robot.BodyNames{end});
    err = norm(T(1:3,4) - target);

    if err < err_thresh && err < topK_err(worst_idx)
        topK_q(:, worst_idx) = q_try;
        topK_err(worst_idx)  = err;
        [~, worst_idx] = max(topK_err);
    end
end

nFound = sum(~isinf(topK_err));
fprintf('Phase 1: %d / %d candidates within %.2fm of target\n', nFound, K, err_thresh);

% Fallback if no candidates found
if nFound == 0
    warning('No configs found within threshold. Using closest point.');
    best_err = inf;
    best_q   = q_hint;
    for s = 1:nSearch
        q_try = jLimits(:,1) + (jLimits(:,2) - jLimits(:,1)) .* rand(nActive,1);
        q_struct = homeConfiguration(robot);
        for j = 1:nActive
            q_struct(j).JointPosition = q_try(j);
        end
        T   = getTransform(robot, q_struct, robot.BodyNames{end});
        err = norm(T(1:3,4) - target);
        if err < best_err
            best_err = err;
            best_q   = q_try;
        end
    end
    J_fb  = computeJacobian(DH_table, best_q, jointTypes);
    w_init = sqrt(abs(det(J_fb(4:6,:) * J_fb(4:6,:)')));
    q_init    = best_q;
    final_err = best_err;
    return;
end

%% --- Phase 2: Select highest manipulability among top-K ---
best_q   = zeros(nActive, 1);
best_w   = -inf;
best_err = inf;

for k = 1:K
    if isinf(topK_err(k)), continue; end
    q_try = topK_q(:, k);
    J     = computeJacobian(DH_table, q_try, jointTypes);
    J_lin = J(4:6, :);
    w     = sqrt(abs(det(J_lin * J_lin')));
    if w > best_w
        best_w   = w;
        best_q   = q_try;
        best_err = topK_err(k);
    end
end

q_init    = best_q;
final_err = best_err;
w_init    = best_w;

fmt = ['Phase 2: err=%.4fm, w=%.4f\nBest q: [' repmat('%.3f, ', 1, nActive)];
fmt = [fmt(1:end-2) ']\n'];
fprintf(fmt, final_err, w_init, q_init');

if final_err > 0.05
    warning('Initial config error is large (%.4fm). IK may drift.', final_err);
end
if w_init < 0.05
    warning('Initial manipulability is low (%.4f). Near singular start.', w_init);
end

end