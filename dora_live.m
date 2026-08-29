% =========================================================================
% DORA Live Mode — Camera → Detect → Plan → Track
% =========================================================================
% This is the live entry point. It connects to the camera, runs the full
% pipeline, saves the session to disk, and starts the tracking loop.
%
% Usage:
%   1. Run dora_setup (once per MATLAB session)
%   2. Run dora_live
% =========================================================================

clear; clc; close all;

%% Load configuration
cfg = dora_config();

%% 1. Camera Setup
cam = init_camera(cfg);

hFig = figure('Name', 'DORA Live — A* Pathfinding', 'Position', [100 100 1000 600]);
disp('Starting pipeline. Close the figure window to stop.');

%% 2. Background Capture
disp('---------------------------------------------------------');
disp('STEP 1: BACKGROUND CAPTURE');
disp('Please CLEAR THE FLOOR (no robot, no markers, no obstacles).');
disp('Capturing background in 3 seconds...');
pause(3);
bgImg_rgb = capture_snapshot(cam);
bgImg = rgb2gray(bgImg_rgb);
disp('Background captured.');

%% 3. Obstacle Capture & Detection
disp('---------------------------------------------------------');
disp('STEP 2: OBSTACLE CAPTURE');
disp('Please PLACE OBSTACLES ONLY (no robot or markers yet).');
disp('Press any key to take the picture...');
pause;
fgImg_rgb = capture_snapshot(cam);
fgImg = rgb2gray(fgImg_rgb);

[cleanObsMask, inflatedObsMask, detectedObstacles] = detect_obstacles(bgImg, fgImg, cfg);
map = build_occupancy_map(inflatedObsMask);

% Visualize occupancy map
figure;
show(map);
title('Oriented Occupancy Grid');
hold on;
for i = 1:length(detectedObstacles)
    plot(detectedObstacles(i).Centroid(1), detectedObstacles(i).Centroid(2), ...
         'r*', 'MarkerSize', 8);
end
hold off;

disp('---------------------------------------------------------');
disp('STATIC MAP GENERATED!');

%% 4. Marker Detection & Pathfinding
disp('Please place the ROBOT and GOAL MARKERS.');
disp('Press any key to take the picture...');
pause;

disp('---------------------------------------------------------');
disp('STEP 3: PATHFINDING');

pathImg = capture_snapshot(cam);
positions = detect_markers(pathImg, [cfg.robotMarkerID, cfg.goalMarkerID], cfg);

% Validate both markers were found
if ~positions.isKey(cfg.robotMarkerID) || ~positions.isKey(cfg.goalMarkerID)
    error('Could not detect both Robot (ID %d) and Goal (ID %d) markers. Restart the script.', ...
          cfg.robotMarkerID, cfg.goalMarkerID);
end

startPos = positions(cfg.robotMarkerID);
goalPos  = positions(cfg.goalMarkerID);

[path, ~] = compute_path(map, startPos, goalPos);

%% 5. Save Session
filepath = save_session(cfg, bgImg_rgb, fgImg_rgb, cleanObsMask, ...
                        inflatedObsMask, detectedObstacles, startPos, goalPos, path);

disp('---------------------------------------------------------');
disp('SETUP COMPLETE. Starting live tracking loop...');
disp('Close the figure window to stop the program.');
disp('---------------------------------------------------------');

%% 6. Live Tracking Loop
while ishandle(hFig)
    % Capture live frame
    img = capture_snapshot(cam);

    % Track robot marker only
    livePositions = detect_markers(img, [cfg.robotMarkerID], cfg);
    currentRobotPos = [];
    if livePositions.isKey(cfg.robotMarkerID)
        currentRobotPos = livePositions(cfg.robotMarkerID);
    end

    % Draw everything
    draw_scene(hFig, img, cleanObsMask, inflatedObsMask, ...
               startPos, goalPos, path, currentRobotPos);
end

clear cam;
disp('Processing finished.');
