function lightTracks = createLightTracks(tracks)
% createLightTracks Create lightweight track structure with essential information only
%
% Description:
%   Extracts and condenses essential information from full track objects into 
%   a lightweight structure for efficient data handling and transmission. 
%   This reduces memory usage and improves performance in real-time systems
%   while preserving critical track data for collision avoidance and planning.
%
% Input Parameters:
%   tracks - Array of full track objects from tracker, typically containing:
%            .State  - Full state vector [px,py,pz,vx,vy,vz,...]
%            .Time   - Track update time
%            (Other tracker-specific fields)
%
% Output Parameters:
%   lightTracks - Lightweight track structure with fields:
%                 .positions  - [N×3] matrix of track positions (x,y,z)
%                 .velocities - [N×3] matrix of track velocities (vx,vy,vz)  
%                 .times      - [N×1] vector of track update times
%
% Algorithm:
%   1. Checks for empty input and returns empty output
%   2. Preallocates output structure for efficiency
%   3. Extracts position (first 3 elements) and velocity (next 3 elements) from state vector
%   4. Copies time information if available in original tracks
%

    % Create lightweight track structure, only saving essential information
    if isempty(tracks)
        lightTracks = [];
        return;
    end
    
    numTracks = length(tracks);
    lightTracks = struct(...
        'positions', zeros(numTracks, 3), ...
        'velocities', zeros(numTracks, 3), ...
        'times', zeros(numTracks, 1));
    
    for i = 1:numTracks
        if length(tracks(i).State) >= 6
            lightTracks.positions(i,:) = tracks(i).State(1:3)';
            lightTracks.velocities(i,:) = tracks(i).State(4:6)';
            
            if isfield(tracks(i), 'Time')
                lightTracks.times(i) = tracks(i).Time;
            end
        end
    end
end