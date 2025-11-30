function refPath = helperCreateReferencePath(waypointsWithTime)
% helperCreateReferencePath Create reference path with adaptive interpolation
%
% Description:
%   Creates a smooth reference path from waypoints using adaptive interpolation
%   based on path length. The function removes duplicate waypoints, computes
%   cumulative arc length, and generates a uniformly sampled path with
%   tangent vectors. Uses piecewise cubic Hermite interpolation for smooth
%   path generation.
%
% Input Parameters:
%   waypointsWithTime - Input waypoints matrix [N×4] where:
%                       Columns 1-3: [x, y, z] coordinates
%                       Column 4: time information (unused in this function)
%
% Output Parameters:
%   refPath - Reference path structure containing:
%             .xyz      - Interpolated path points [M×3]
%             .s        - Cumulative arc length at each point [M×1]
%             .tangents - Unit tangent vectors at each point [M×3]
%             .sMax     - Total path length (meters)
%
% Algorithm:
%   1. Remove consecutive duplicate waypoints to prevent zero-length segments
%   2. Calculate total path length and determine adaptive number of points
%   3. Generate uniform arc length sampling
%   4. Interpolate positions using piecewise cubic Hermite interpolation
%   5. Compute tangent vectors using central differences
%
% Features:
%   - Adaptive point density: 1 point per 5 meters of path length
%   - Minimum 50 points to ensure adequate sampling
%   - Robust handling of degenerate cases (single point, zero length)
%   - Smooth tangent calculation with edge case handling

    % Create reference path (adaptive interpolation points)
    
    waypoints = waypointsWithTime(:, 1:3);

    % Remove consecutive duplicate points to prevent zero-length segments
    if size(waypoints,1) > 1
        diffs = vecnorm(diff(waypoints,1,1),2,2);
        keep = [true; diffs > 1e-6];
        waypoints = waypoints(keep,:);
    end
    
    if size(waypoints,1) < 2
        % Degenerate case: only one point
        refPath = struct();
        refPath.s = 0;
        refPath.xyz = waypoints;
        refPath.tangents = [1 0 0];
        refPath.sMax = 0;
        return;
    end
    
    % Calculate total path length
    segmentLengths = vecnorm(diff(waypoints, 1, 1), 2, 2);
    totalLength = sum(segmentLengths);
    
    % One point every 5 meters (instead of fixed 200 points)
    numPoints = max(50, ceil(totalLength / 5));
    
    % Create path parameter s
    s = [0; cumsum(segmentLengths)];
    sMax = s(end);
    if sMax < 1e-6
        sMax = 1e-6;
        s(end) = sMax;
    end
    
    % Uniform sampling
    sQuery = linspace(0, sMax, numPoints);
    
    % Interpolate positions
    xyz = interp1(s, waypoints, sQuery, 'pchip');
    
    % Calculate tangents
    tangents = zeros(numPoints, 3);
    for i = 1:numPoints
        if i == 1
            v = xyz(2, :) - xyz(1, :);
        elseif i == numPoints
            v = xyz(end, :) - xyz(end-1, :);
        else
            v = xyz(i+1, :) - xyz(i-1, :);
        end
        n = norm(v);
        if n < 1e-6
            tangents(i,:) = [1 0 0];
        else
            tangents(i,:) = v / n;
        end
    end
    
    % Return reference path
    refPath = struct();
    refPath.s = sQuery';
    refPath.xyz = xyz;
    refPath.tangents = tangents;
    refPath.sMax = sMax;
end