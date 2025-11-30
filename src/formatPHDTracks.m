function tracksout = formatPHDTracks(tracksin)
% formatPHDTracks Convert PHD track format to standard objectTrack format
%
% Description:
%   Converts tracks from Probability Hypothesis Density (PHD) filter format 
%   to standard objectTrack format used by MATLAB's tracking system. 
%   This function extracts both kinematic state and extended target 
%   information (dimensions and orientation) for better object representation.
%
% Input Parameters:
%   tracksin - Input tracks in PHD filter format, typically containing:
%              .State - Kinematic state vector
%              .StateCovariance - State covariance matrix  
%              .Extent - Target extent information
%
% Output Parameters:
%   tracksout - Output tracks in standard objectTrack format with:
%               .State - Extended state vector [kinematic; dimensions; quaternion]
%               .StateCovariance - Extended covariance matrix
%               All other objectTrack properties
%
% Algorithm:
%   1. Creates objectTrack array of same size as input
%   2. Copies basic track properties
%   3. Converts extended track information (dimensions and orientation)
%   4. Updates state and covariance with extended information

    % Convert PHD track format
    N = numel(tracksin);
    tracksout = repmat(objectTrack, N, 1);
    for i = 1:N
        tracksout(i) = objectTrack(tracksin(i));
        [state, statecov] = convertExtendedTrack(tracksin(i));
        tracksout(i).State = state;
        tracksout(i).StateCovariance = statecov;
    end
end

function [state, statecov] = convertExtendedTrack(track)
% convertExtendedTrack Convert extended track information to state vector
%
% Description:
%   Extends the standard kinematic state with target dimension and orientation
%   information derived from the track's extent matrix. This provides a more
%   complete representation of the tracked object's physical properties.
%
% Input Parameters:
%   track - Input track with extent information
%
% Output Parameters:
%   state - Extended state vector [kinematic_state; dimensions; quaternion]
%   statecov - Extended covariance matrix with blocks for kinematics, dimensions, and orientation

    % Extended state information
    extent = track.Extent;
    [V, D] = eig(extent);
    [dims, idx] = sort(1.5*sqrt(diag(D)), 'descend');
    V = V(:, idx);
    q = quaternion(V, 'rotmat', 'frame');
    q = q./norm(q);
    [q1, q2, q3, q4] = parts(q);
    state = [track.State; dims(:); q1; q2; q3; q4];
    statecov = blkdiag(track.StateCovariance, 4*eye(3), 4*eye(4));
end