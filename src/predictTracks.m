function predictedTracks = predictTracks(tracks, predictionHorizon, dt)
% predictTracks - Predict future trajectories of tracked objects
%
% Input:
%   tracks - Current tracked objects (objectTrack array)
%   predictionHorizon - Prediction time horizon (seconds), default 5.0
%   dt - Time step (seconds), default 0.1
%
% Output:
%   predictedTracks - Predicted trajectory structure array
%       .TrackID - Track ID
%       .Trajectory - Predicted trajectory points [N x 6], each row [x, y, z, vx, vy, vz]
%       .Time - Corresponding time points [N x 1]
%       .CurrentState - Current state at present time [1 x 6]
%
% Example:
%   predictedTracks = predictTracks(allTracks, 5.0, 0.1);

    % Default parameter values
    if nargin < 2
        predictionHorizon = 5.0;
    end
    if nargin < 3
        dt = 0.1;
    end
    
    % Initialize output
    predictedTracks = struct('TrackID', {}, 'Trajectory', {}, 'Time', {}, 'CurrentState', {});
    
    % Return immediately if no tracks
    if isempty(tracks)
        return;
    end
    
    % Calculate number of prediction steps
    numSteps = ceil(predictionHorizon / dt);
    
    % Predict each tracked object
    for i = 1:length(tracks)
        track = tracks(i);
        
        % Get current state
        state = track.State;
        
        % Extract position and velocity
        if length(state) >= 6
            pos = state(1:3);    % [x, y, z]
            vel = state(4:6);    % [vx, vy, vz]
        elseif length(state) >= 3
            % If state only has position, assume zero velocity
            pos = state(1:3);
            vel = [0, 0, 0];
            warning('Track %d state length insufficient, assuming zero velocity', track.TrackID);
        else
            warning('Track %d state length insufficient, skipping prediction', track.TrackID);
            continue;
        end
        
        % Initialize predicted trajectory
        trajectory = zeros(numSteps, 6);
        timeSteps = zeros(numSteps, 1);
        
        % Use constant velocity linear motion model for prediction
        for j = 1:numSteps
            t = j * dt;
            
            % Predict position: p(t) = p0 + v * t
            trajectory(j, 1:3) = pos + vel * t;
            
            % Predict velocity: assume constant velocity
            trajectory(j, 4:6) = vel;
            
            % Timestamp
            timeSteps(j) = t;
        end
        
        % Save prediction results
        predictedTracks(end+1) = struct(...
            'TrackID', track.TrackID, ...
            'Trajectory', trajectory, ...
            'Time', timeSteps, ...
            'CurrentState', [pos, vel]);
    end
end