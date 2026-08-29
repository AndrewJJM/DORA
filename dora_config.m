function cfg = dora_config()
%DORA_CONFIG  Centralized configuration for the DORA BEV pipeline.
%   cfg = dora_config() returns a struct containing every tunable
%   parameter used across the pipeline modules.
%
%   Edit this file to change thresholds, marker IDs, camera settings,
%   etc.  All other modules receive 'cfg' as an input argument.

    % --- Camera ---
    cfg.useMobileCamera  = true;       % true = MATLAB Mobile, false = webcam
    cfg.camResolution    = '640x480';  % Resolution string for mobile camera

    % --- Marker IDs ---
    cfg.startMarkerID    = 0;
    cfg.goalMarkerID     = 1;
    cfg.robotMarkerID    = 0;
    cfg.markerDict       = 'DICT_4X4_50';

    % --- Obstacle Detection ---
    cfg.diffThreshold    = 10;    % Background‐subtraction threshold (0–255)
    cfg.minObstacleArea  = 400;   % Min pixel area to keep a region
    cfg.morphDiskRadius  = 3;     % Structuring element radius for cleaning

    % --- Occupancy Map & Inflation ---
    cfg.robotRadiusPx    = 10;    % Obstacle inflation radius (pixels)

    % --- Pathfinding ---
    cfg.goalReachedDist  = 20;    % Goal‐reached threshold (pixels)

    % --- I/O ---
    cfg.dataDir = fullfile(fileparts(mfilename('fullpath')), 'data');
end
