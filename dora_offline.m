% =========================================================================
% DORA Offline Mode — Load Saved Data → Re-run Detection/Planning/Viz
% =========================================================================
% This is the offline entry point. It loads a previously saved session
% and lets you re-run any pipeline stage with modified parameters,
% without needing a camera.
%
% Usage:
%   1. Run dora_setup (once per MATLAB session)
%   2. Run dora_offline
%      — If called with no session file, it will list available sessions
%        and prompt you to choose one.
%
% Workflow:
%   - Edit dora_config.m to tweak thresholds / parameters
%   - Run dora_offline to instantly see the effect on saved images
% =========================================================================

clear; clc; close all;

%% Load configuration (potentially with modified parameters)
cfg = dora_config();

%% 1. Load a saved session
% Call load_session() with no arguments to get an interactive picker,
% or pass a specific file path:
%   session = load_session('data/session_20260829_141500.mat');
session = load_session();

%% 2. Re-run obstacle detection with current config
% This is the key benefit: change cfg.diffThreshold, cfg.minObstacleArea,
% etc. in dora_config.m and re-run this section to see the effect.
disp('---------------------------------------------------------');
disp('Re-running obstacle detection with current config...');
[cleanObsMask, inflatedObsMask, detectedObstacles] = ...
    detect_obstacles(session.bgImg, session.fgImg, cfg);

fprintf('Detected %d obstacles.\n', length(detectedObstacles));

%% 3. Build occupancy map
map = build_occupancy_map(inflatedObsMask);

% Visualize occupancy grid
figure('Name', 'Occupancy Grid');
show(map);
title('Occupancy Grid (Offline)');
hold on;
for i = 1:length(detectedObstacles)
    plot(detectedObstacles(i).Centroid(1), detectedObstacles(i).Centroid(2), ...
         'r*', 'MarkerSize', 8);
end
hold off;

%% 4. Re-plan path
disp('---------------------------------------------------------');
disp('Re-planning A* path...');
[path, ~] = compute_path(map, session.startPos, session.goalPos);

%% 5. Visualize the full scene
hFig = figure('Name', 'DORA Offline — A* Pathfinding', 'Position', [100 100 1000 600]);
draw_scene(hFig, session.fgImg_rgb, cleanObsMask, inflatedObsMask, ...
           session.startPos, session.goalPos, path, []);

disp('---------------------------------------------------------');
disp('Offline analysis complete.');
disp('Tweak dora_config.m and re-run dora_offline to iterate.');
