function positions = detect_markers(img, markerIDs, cfg)
%DETECT_MARKERS  Detect ArUco markers and return their center positions.
%
%   positions = detect_markers(img, markerIDs, cfg)
%
%   Inputs:
%     img       - RGB image to scan for markers
%     markerIDs - vector of marker IDs to look for (e.g. [0, 1])
%     cfg       - config struct from dora_config()
%
%   Outputs:
%     positions - containers.Map (double keys) mapping markerID → [x, y].
%                 Only contains entries for markers that were actually found.
%
%   Example:
%     pos = detect_markers(img, [0 1], cfg);
%     if pos.isKey(0)
%         robotXY = pos(0);
%     end

    positions = containers.Map('KeyType', 'double', 'ValueType', 'any');

    try
        [detectedIDs, detectedLocs] = readArucoMarker(img, cfg.markerDict);
    catch
        % No markers found — return empty map
        return;
    end

    if isempty(detectedIDs)
        return;
    end

    for i = 1:length(detectedIDs)
        id = detectedIDs(i);
        if ismember(id, markerIDs)
            corners = detectedLocs(:,:,i);
            centerX = mean(corners(:,1));
            centerY = mean(corners(:,2));
            positions(id) = [centerX, centerY];
        end
    end
end
