function map = build_occupancy_map(inflatedObsMask)
%BUILD_OCCUPANCY_MAP  Create a binaryOccupancyMap from an inflated obstacle mask.
%
%   map = build_occupancy_map(inflatedObsMask)
%
%   Inputs:
%     inflatedObsMask - logical matrix where true = occupied (inflated obstacles)
%
%   Outputs:
%     map - binaryOccupancyMap object ready for path planning

    map = binaryOccupancyMap(inflatedObsMask);
end
