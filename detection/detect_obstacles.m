function [cleanObsMask, inflatedObsMask, detectedObstacles] = detect_obstacles(bgImg, fgImg, cfg)
%DETECT_OBSTACLES  Background subtraction → morphology → convex‐hull obstacle extraction.
%
%   [cleanObsMask, inflatedObsMask, detectedObstacles] = detect_obstacles(bgImg, fgImg, cfg)
%
%   Inputs:
%     bgImg  - grayscale background image (empty floor, no obstacles)
%     fgImg  - grayscale foreground image (obstacles placed on floor)
%     cfg    - config struct from dora_config()
%
%   Outputs:
%     cleanObsMask      - binary mask of exact obstacle boundaries (convex hulls)
%     inflatedObsMask   - mask inflated by cfg.robotRadiusPx for path planning
%     detectedObstacles - regionprops struct array of valid obstacles
%                         (fields: Area, ConvexHull, Centroid, Orientation, BoundingBox)

    % --- Background Subtraction ---
    diffImg = imabsdiff(fgImg, bgImg);

    % Threshold to ignore soft shadows
    bwObs = diffImg > cfg.diffThreshold;

    % --- Shadow Reduction & Cleanup ---
    se_clean = strel('disk', cfg.morphDiskRadius);
    bwObs = imopen(bwObs, se_clean);

    % Remove small disconnected specks (noise)
    bwObs = bwareaopen(bwObs, cfg.minObstacleArea);
    bwObs = imfill(bwObs, 'holes');

    % --- Extract obstacle regions ---
    stats = regionprops(bwObs, 'Area', 'ConvexHull', 'Centroid', 'Orientation', 'BoundingBox');

    [imgH, imgW] = size(bwObs);
    cleanObsMask = false(imgH, imgW);
    detectedObstacles = [];

    for k = 1:length(stats)
        if stats(k).Area > cfg.minObstacleArea
            % Save valid obstacle
            detectedObstacles = [detectedObstacles; stats(k)]; %#ok<AGROW>

            % Create mask from convex hull (preserves rotation / exact shape)
            hull = stats(k).ConvexHull;
            polyMask = poly2mask(hull(:,1), hull(:,2), imgH, imgW);
            cleanObsMask = cleanObsMask | polyMask;
        end
    end

    % --- Inflate obstacles by robot radius ---
    se_inflate = strel('disk', cfg.robotRadiusPx);
    inflatedObsMask = imdilate(cleanObsMask, se_inflate);
end
