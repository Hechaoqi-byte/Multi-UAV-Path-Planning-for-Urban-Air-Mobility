function tangents = calculateTangents(pathXYZ)
% calculateTangents Calculate unit tangent vectors for each point on the path, called in calculateTrajectoryCosts.m
%
% Description:
%   Given a discrete path point sequence pathXYZ (N×3), returns the tangent vector (unit vector) at each point.
%   The output can be used for velocity direction, normal vector calculation, or trajectory visualization.
%
% Input:
%   pathXYZ - [N x 3] Path point coordinates, each row is [x y z]
%
% Output:
%   tangents - [N x 3] Unit tangent vector corresponding to each point
%
% Notes:
%   - Uses forward difference to calculate tangent vectors: t_i = p_{i+1} - p_i (i=1..N-1)
%   - The tangent vector of the last point takes the previous point's tangent vector to ensure output row count matches input
%   - For degenerate or zero-length segments, returns default vector [1 0 0]

    % Initialize
    n = size(pathXYZ, 1);
    if n < 1
        tangents = zeros(0, 3);
        return;
    end

    tangents = zeros(n, 3);

    % When there's only 1 point, return default tangent vector
    if n < 2
        tangents(1, :) = [1, 0, 0];
        return;
    end

    % Forward difference calculation and normalization
    for i = 1:(n-1)
        v = pathXYZ(i+1, :) - pathXYZ(i, :);
        normV = norm(v);
        if normV > 1e-6
            tangents(i, :) = v / normV;
        else
            % Avoid division by zero: return default direction for degenerate segments
            tangents(i, :) = [1, 0, 0];
        end
    end

    % Last point reuses the previous tangent vector (avoid special boundary handling)
    tangents(end, :) = tangents(end-1, :);
end