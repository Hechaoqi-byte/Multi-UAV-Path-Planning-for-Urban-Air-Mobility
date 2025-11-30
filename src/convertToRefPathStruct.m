function refPathStruct = convertToRefPathStruct(pathArray)
% convertToRefPathStruct Convert path array to standardized reference path structure
%
% Description:
%   Transforms a simple [N×3] path coordinate array into a comprehensive 
%   reference path structure containing position data, arc length parameters,
%   and tangent vectors. This standardized format is used throughout the 
%   trajectory planning system for consistent path representation.
%
% Input Parameters:
%   pathArray - Input path data, which can be:
%               - [N×3] matrix of xyz coordinates
%               - Existing refPathStruct (returned unchanged)
%               - Empty array or invalid data
%
% Output Parameters:
%   refPathStruct - Standardized reference path structure with fields:
%                   .xyz      - [N×3] path coordinates
%                   .s        - [N×1] cumulative arc length from start (meters)
%                   .sMax     - Total path length (meters)
%                   .tangents - [N×3] unit tangent vectors at each point
%
% Algorithm:
%   1. Validates input and handles edge cases (empty, struct, invalid)
%   2. Computes cumulative arc length using Euclidean distances
%   3. Calculates unit tangent vectors via forward differences
%   4. Ensures consistent tangent direction for single-point paths
%
% Examples:
%   % Convert coordinate array to path structure
%   pathCoords = [0,0,0; 1,0,0; 1,1,0; 2,1,0];
%   refPath = convertToRefPathStruct(pathCoords);
%
%   % Handle existing structure (no conversion needed)
%   refPath2 = convertToRefPathStruct(refPath);

    % Convert [N×3] path array to standard structure
    % Input: pathArray - [N×3] xyz coordinates
    % Output: refPathStruct - structure containing xyz, s, tangents
    
    if isstruct(pathArray)
        % Already a structure, return directly
        refPathStruct = pathArray;
        return;
    end
    
    if isempty(pathArray) || size(pathArray, 2) < 3
        % Empty path or incorrect dimensions
        refPathStruct = struct('xyz', [], 's', [], 'tangents', [], 'sMax', 0);
        return;
    end
    
    % Create structure
    refPathStruct = struct();
    refPathStruct.xyz = pathArray;
    
    % Calculate arc length
    segLengths = [0; vecnorm(diff(pathArray, 1, 1), 2, 2)];
    refPathStruct.s = cumsum(segLengths);
    refPathStruct.sMax = refPathStruct.s(end);
    
    % Calculate tangent vectors
    n = size(pathArray, 1);
    tangents = zeros(n, 3);
    
    for i = 1:n-1
        tangents(i, :) = pathArray(i+1, :) - pathArray(i, :);
        tangentNorm = norm(tangents(i, :));
        if tangentNorm > 1e-6
            tangents(i, :) = tangents(i, :) / tangentNorm;
        else
            tangents(i, :) = [1, 0, 0];
        end
    end
    % Last point uses previous point's tangent vector
    if n > 1
        tangents(end, :) = tangents(end-1, :);
    else
        tangents(end, :) = [1, 0, 0];
    end
    
    refPathStruct.tangents = tangents;
end