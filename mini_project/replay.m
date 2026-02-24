% replay.m
% Replay the welding simulation without rerunning the IK loop.
% Run main.m first to generate sim_results.mat

clc; clear; close all;

%% --- Load Results ---
if ~isfile('sim_results.mat')
    error('sim_results.mat not found. Run main.m first.');
end

load('sim_results.mat');
fprintf('Loaded simulation results.\n');
fprintf('  Timesteps : %d\n', path.N);
fprintf('  dt        : %.3f s\n', path.dt);
fprintf('  Config    : %s\n\n', configStr);

%% --- Rebuild Robot ---
[robot, ~, ~] = buildConfig(configStr);

%% --- Replay ---
visualize(robot, path, q_hist, qdot_hist, ee_hist, w_hist, lambda_hist, baseOffset);