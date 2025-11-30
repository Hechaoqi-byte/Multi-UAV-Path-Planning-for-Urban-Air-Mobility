function length_total = calculateTrajectoryLength(traj)
% calculateTrajectoryLength Calculate total length of a trajectory or path
%
% Description:
%   Computes the cumulative Euclidean distance along a trajectory by summing
%   the distances between consecutive points. Handles both pure position
%   trajectories and state trajectories containing additional columns.
%
% Input Parameters:
%   traj - Trajectory matrix [N×M] where:
%          N: number of points
%          M: number of columns (must be at least 3)
%          First 3 columns are interpreted as [x, y, z] coordinates
%          Can be either:
%          - Pure position trajectory [N×3] 
%          - State trajectory [N×≥3] with positions in first 3 columns
%
% Output Parameters:
%   length_total - Total path length in meters (scalar)
%
% Examples:
%   % Pure position trajectory
%   length = calculateTrajectoryLength([0,0,0; 1,0,0; 1,1,0]);
%
%   % State trajectory with velocity information
%   length = calculateTrajectoryLength([0,0,0,1,0,0; 1,0,0,0,1,0]);

    % Handle empty or insufficient points
    if isempty(traj) || size(traj,1) < 2
        length_total = 0;
        return;
    end

    % Extract first 3 columns as position data
    if size(traj,2) >= 3
        pos = traj(:,1:3);
    else
        error('calculateTrajectoryLength: Input must have at least 3 columns (x,y,z)');
    end

    % Calculate displacement vectors between consecutive points
    d = diff(pos,1,1);      % [N-1 × 3]
    seg_len = vecnorm(d,2,2);  % Segment lengths
    length_total = sum(seg_len);
end