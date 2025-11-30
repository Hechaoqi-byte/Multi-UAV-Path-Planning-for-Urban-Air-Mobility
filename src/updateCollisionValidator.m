function validator = updateCollisionValidator(validator, egoState, gridMap, tracks, time)
% updateCollisionValidator Update collision validator with obstacle predictions
%
% Description:
%   Updates the collision validator by predicting future positions of tracked
%   obstacles using linear motion models. This function maintains a frequent
%   update cycle to ensure real-time collision risk assessment and supports
%   both 6D and 3D state vectors from different tracking systems.
%
% Input Parameters:
%   validator - Collision validator structure to be updated
%   egoState  - Ego vehicle state vector [x,y,z,vx,vy,vz,...] (currently unused)
%   gridMap   - Occupancy grid map for static obstacles (currently unused)
%   tracks    - Array of tracked obstacle objects containing:
%               .State - State vector [x,vx,y,vy,z,vz] or [x,y,z]
%   time      - Current simulation time for update timing control
%
% Output Parameters:
%   validator - Updated collision validator structure with:
%               .predictedObstacles - Cell array of predicted obstacle trajectories
%               .lastUpdateTime     - Timestamp of last update
%
% Algorithm:
%   1. Checks update frequency (0.15s interval) to avoid excessive computation
%   2. Processes each track to extract position and velocity information
%   3. Uses linear motion model to predict future positions over prediction horizon
%   4. Stores predicted trajectories for collision checking
%
% Prediction Parameters:
%   - Prediction Horizon: 3.0 seconds (default, configurable)
%   - Time Step: 0.1 seconds
%   - Update Interval: 0.15 seconds minimum
%
% State Vector Support:
%   - 6D states: [x, vx, y, vy, z, vz] - Full kinematic information
%   - 3D states: [x, y, z] - Position only, assumes zero velocity

    % Update collision validator state
    
    % ★★★ Fix: More frequent updates ★★★
    if isfield(validator, 'lastUpdateTime') && (time - validator.lastUpdateTime < 0.15)
        return;
    end
    
    validator.predictedObstacles = {};
    
    if isempty(tracks)
        validator.lastUpdateTime = time;
        return;
    end
    
    if isfield(validator, 'predictionHorizon')
        T = validator.predictionHorizon;
    else
        T = 3.0;
    end
    
    dt = 0.1;
    numSteps = round(T / dt) + 1;
    times = linspace(0, T, numSteps);
    
    for i = 1:length(tracks)
        state = tracks(i).State;
        
        if length(state) >= 6
            pos = [state(1); state(3); state(5)];
            vel = [state(2); state(4); state(6)];
        elseif length(state) >= 3
            pos = state(1:3);
            if size(pos, 2) > size(pos, 1)
                pos = pos';
            end
            vel = [0; 0; 0];
        else
            continue;
        end
        
        if size(pos, 2) > size(pos, 1)
            pos = pos';
        end
        if size(vel, 2) > size(vel, 1)
            vel = vel';
        end
        
        futurePos = zeros(numSteps, 3);
        for t = 1:numSteps
            futurePos(t, :) = (pos + vel * times(t))';
        end
        
        validator.predictedObstacles{end+1} = futurePos;
    end
    
    validator.lastUpdateTime = time;
end