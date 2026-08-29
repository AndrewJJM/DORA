function [path, planner] = compute_path(map, startPos, goalPos)
%COMPUTE_PATH  Plan an A* path on the given occupancy map.
%
%   [path, planner] = compute_path(map, startPos, goalPos)
%
%   Inputs:
%     map      - binaryOccupancyMap from build_occupancy_map()
%     startPos - [x, y] start position (pixels)
%     goalPos  - [x, y] goal position  (pixels)
%
%   Outputs:
%     path    - Nx2 array of [x, y] waypoints, or empty [] if no path found
%     planner - plannerAStarGrid object (reusable for replanning)

    % Clamp positions to map bounds
    gridSize = map.GridSize;  % [rows, cols]
    startPt = round(startPos);
    goalPt  = round(goalPos);

    startPt(1) = max(1, min(startPt(1), gridSize(2)));
    startPt(2) = max(1, min(startPt(2), gridSize(1)));
    goalPt(1)  = max(1, min(goalPt(1),  gridSize(2)));
    goalPt(2)  = max(1, min(goalPt(2),  gridSize(1)));

    % Create planner and plan
    planner = plannerAStarGrid(map);
    path = [];

    try
        path = plan(planner, startPt, goalPt);
        disp('A* path calculated successfully.');
    catch ME
        fprintf('WARNING: No valid path found — %s\n', ME.message);
    end
end
