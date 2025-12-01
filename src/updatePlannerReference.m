function planner = updatePlannerReference(planner, newTrajectory) 
% updatePlannerReference Update planner's reference path with new trajectory
%
% Description:
%   Updates the reference path in the planner structure with a new trajectory.
%   Converts raw trajectory coordinates into a standardized reference path
%   structure containing position data, arc length parameters, and tangent
%   vectors. This enables dynamic path updates during mission execution.
%
% Input Parameters:
%   planner      - Planner structure containing existing configuration
%   newTrajectory- New trajectory data [N×M] where:
%                  First 3 columns are [x, y, z] coordinates
%                  Additional columns may contain velocity, acceleration, etc.
%
% Output Parameters:
%   planner      - Updated planner structure with:
%                  .refPath - Updated reference path structure containing:
%                             .xyz      - Path coordinates [N×3]
%                             .s        - Cumulative arc length [N×1]
%                             .tangents - Unit tangent vectors [N×3]
%                             .sMax     - Total path length
%
% Algorithm:
%   1. Validates planner structure contains refPath field
%   2. Extracts position coordinates from new trajectory (first 3 columns)
%   3. Converts coordinates to standardized reference path structure
%   4. Updates planner's refPath field with new reference path
%
% Error Handling:
%   - Issues warning if planner structure is incompatible
%   - Uses robust conversion function to handle various trajectory formats

    % Update planner's reference path
    
    if isfield(planner, 'refPath')
        % Use our general function to convert structure
        refPath_new = convertToRefPathStruct(newTrajectory(:, 1:3));
        
        % Replace old refPath
        planner.refPath = refPath_new;
        
        % fprintf('    Planner reference path updated (%.1fm)\n', refPath_new.sMax);
    else
        warning('Planner structure incompatible, cannot update reference path');
    end
end