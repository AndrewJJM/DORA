function draw_scene(hFig, img, cleanObsMask, inflatedObsMask, ...
                    startPos, goalPos, path, currentRobotPos)
%DRAW_SCENE  Render the full DORA scene: image, obstacles, path, markers.
%
%   draw_scene(hFig, img, cleanObsMask, inflatedObsMask, ...
%              startPos, goalPos, path, currentRobotPos)
%
%   Inputs:
%     hFig            - figure handle (will be clf'd before drawing)
%     img             - RGB image to display as background
%     cleanObsMask    - binary mask of exact obstacle boundaries
%     inflatedObsMask - binary mask of inflated safety boundaries
%     startPos        - [x, y] start/robot-initial position
%     goalPos         - [x, y] goal position
%     path            - Nx2 array of waypoints, or [] if no path
%     currentRobotPos - [x, y] live robot position, or [] if not tracked

    clf(hFig);
    imshow(img);
    hold on;

    % 1. Draw exact obstacle boundaries (red solid)
    obsBoundaries = bwboundaries(cleanObsMask);
    for k = 1:length(obsBoundaries)
        b = obsBoundaries{k};
        plot(b(:,2), b(:,1), 'r-', 'LineWidth', 2);
    end

    % 2. Draw inflated safety boundaries (yellow dashed)
    infBoundaries = bwboundaries(inflatedObsMask);
    for k = 1:length(infBoundaries)
        b = infBoundaries{k};
        plot(b(:,2), b(:,1), 'y--', 'LineWidth', 1);
    end

    % 3. Draw START position
    if ~isempty(startPos)
        plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 3);
        text(startPos(1)-20, startPos(2)-20, 'START', ...
             'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
    end

    % 4. Draw GOAL position
    if ~isempty(goalPos)
        plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 3);
        text(goalPos(1)-20, goalPos(2)-20, 'GOAL', ...
             'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    end

    % 5. Draw A* path (cyan)
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'c-', 'LineWidth', 4);
    end

    % 6. Draw LIVE robot position (magenta)
    if ~isempty(currentRobotPos)
        plot(currentRobotPos(1), currentRobotPos(2), 'mo', ...
             'MarkerSize', 12, 'LineWidth', 4);
        text(currentRobotPos(1)+15, currentRobotPos(2), 'ROBOT', ...
             'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
    end

    title('DORA — Real-Time A* Pathfinding');
    hold off;
    drawnow;
end
