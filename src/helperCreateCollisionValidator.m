function validator = helperCreateCollisionValidator(gridTracker, vehDims)
% helperCreateCollisionValidator Create and initialize collision validator structure
%
% Description:
%   Creates a collision validator structure for trajectory validation in 
%   dynamic environments. The validator predicts obstacle positions over time,
%   calculates collision probabilities for candidate trajectories, and 
%   implements safety margin-based collision detection with probabilistic
%   risk assessment.
%
% Input Parameters:
%   gridTracker - Grid-based tracker object for obstacle tracking
%   vehDims     - Vehicle dimensions structure containing physical size info
%
% Output Parameters:
%   validator - Collision validator structure with fields:
%               .gridTracker        - Grid tracker reference
%               .vehDims            - Vehicle dimensions
%               .safetyMargin       - Safety distance margin (meters)
%               .predictionHorizon  - Obstacle prediction time (seconds)
%               .predictedObstacles - Cell array of predicted obstacle positions
%               .lastUpdateTime     - Last update timestamp
%               .tracker            - Tracker object (optional)
%               .obstacleVelocities - Cell array of obstacle velocity vectors
%
% Components:
%   - Collision prediction with linear obstacle motion models
%   - Probabilistic collision risk assessment
%   - Closest Point of Approach (CPA) calculation
%   - Safety margin-based validation

    % Create collision validator
    
    validator = struct();
    validator.gridTracker = gridTracker;
    validator.vehDims = vehDims;
    validator.safetyMargin = 10;          % Safety distance (meters)
    validator.predictionHorizon = 3.0;    % Prediction time (seconds)
    validator.predictedObstacles = {};    % Changed to cell array
    validator.lastUpdateTime = -inf;      % Initialized to -inf to ensure first update
    validator.tracker = [];
    
    % ★★★ New: Obstacle velocity information ★★★
    validator.obstacleVelocities = {};
end

%% ============================================================
%% updateCollisionValidator - Fixed Version
%% ============================================================
function validator = updateCollisionValidator(validator, egoState, gridMap, tracks, time)
% updateCollisionValidator Update validator state with obstacle predictions
%
% Description:
%   Updates the collision validator by predicting future obstacle positions
%   based on current track states. Uses linear motion models to forecast
%   obstacle trajectories over the prediction horizon for collision checking.
%
% Input Parameters:
%   validator - Collision validator structure
%   egoState  - Ego vehicle state vector [x,y,z,vx,vy,vz,...]
%   gridMap   - Occupancy grid map for static obstacle checking
%   tracks    - Array of tracked obstacle objects
%   time      - Current simulation time
%
% Output Parameters:
%   validator - Updated collision validator structure

    % Update validator state (predict obstacle positions)
    
    % ★★★ Fix: Reduce update interval, update more frequently ★★★
    if time - validator.lastUpdateTime < 0.2
        return;
    end
    
    validator.predictedObstacles = {};
    validator.obstacleVelocities = {};
    
    if isempty(tracks)
        validator.lastUpdateTime = time;
        return;
    end
    
    % Prediction time and step size
    T = validator.predictionHorizon;
    dt = 0.1;
    numSteps = round(T / dt) + 1;
    times = linspace(0, T, numSteps);
    
    for i = 1:length(tracks)
        state = tracks(i).State;
        
        if length(state) >= 6
            % 6D state: [x, vx, y, vy, z, vz]
            pos = [state(1); state(3); state(5)];
            vel = [state(2); state(4); state(6)];
        elseif length(state) >= 3
            pos = state(1:3);
            vel = [0; 0; 0];
        else
            continue;
        end
        
        % Linear prediction of future positions
        futurePos = zeros(numSteps, 3);
        for t = 1:numSteps
            futurePos(t, :) = (pos + vel * times(t))';
        end
        
        validator.predictedObstacles{end+1} = futurePos;
        validator.obstacleVelocities{end+1} = vel';
    end
    
    validator.lastUpdateTime = time;
end

%% ============================================================
%% validateTrajectories - Fixed Version: More accurate collision probability calculation
%% ============================================================
function [isValid, collisionProb] = validateTrajectories(validator, trajectories)
% validateTrajectories Validate trajectories against predicted obstacles
%
% Description:
%   Checks candidate trajectories for potential collisions with predicted
%   obstacle positions. Returns validation results and collision probabilities
%   based on minimum distance calculations and safety margins.
%
% Input Parameters:
%   validator     - Collision validator structure
%   trajectories  - Cell array of candidate trajectories, each [N×14]
%
% Output Parameters:
%   isValid      - Logical vector indicating valid trajectories
%   collisionProb- Vector of collision probabilities [0-1] for each trajectory

    % Validate trajectories for collisions with obstacles
    
    numTrajs = length(trajectories);
    isValid = true(numTrajs, 1);
    collisionProb = zeros(numTrajs, 1);
    
    % If no obstacle predictions, all trajectories are valid
    if isempty(validator.predictedObstacles)
        return;
    end
    
    safetyMargin = validator.safetyMargin;
    
    for i = 1:numTrajs
        traj = trajectories{i};
        
        if isempty(traj) || size(traj, 1) < 2
            isValid(i) = false;
            collisionProb(i) = 1.0;
            continue;
        end
        
        trajPos = traj(:, 1:3);
        minDist = inf;
        
        % Check distance to each obstacle
        for j = 1:length(validator.predictedObstacles)
            obsPred = validator.predictedObstacles{j};
            
            if isempty(obsPred)
                continue;
            end
            
            % ★★★ Fix: Calculate minimum distance along entire trajectory ★★★
            numSteps = min(size(trajPos, 1), size(obsPred, 1));
            
            for t = 1:numSteps
                dist = norm(trajPos(t, :) - obsPred(t, :));
                minDist = min(minDist, dist);
            end
            
            % ★★★ New: Additional check for adjacent time steps (prevent missed collisions) ★★★
            if numSteps > 1
                for t = 1:numSteps-1
                    % Check intermediate points
                    midTraj = (trajPos(t,:) + trajPos(t+1,:)) / 2;
                    midObs = (obsPred(t,:) + obsPred(min(t+1,size(obsPred,1)),:)) / 2;
                    dist = norm(midTraj - midObs);
                    minDist = min(minDist, dist);
                end
            end
        end
        
        % ★★★ Fix: More detailed collision probability calculation ★★★
        if minDist < safetyMargin * 0.5
            % Very dangerous: certain collision
            isValid(i) = false;
            collisionProb(i) = 1.0;
        elseif minDist < safetyMargin
            % Dangerous: high collision probability
            isValid(i) = false;
            collisionProb(i) = 0.5 + 0.5 * (1 - minDist / safetyMargin);
        elseif minDist < safetyMargin * 1.5
            % Warning: medium collision probability
            isValid(i) = true;
            collisionProb(i) = 0.3 * (1 - (minDist - safetyMargin) / (safetyMargin * 0.5));
        elseif minDist < safetyMargin * 2.0
            % Attention: low collision probability
            isValid(i) = true;
            collisionProb(i) = 0.1 * (1 - (minDist - safetyMargin * 1.5) / (safetyMargin * 0.5));
        else
            % Safe
            isValid(i) = true;
            collisionProb(i) = 0;
        end
    end
end

%% ============================================================
%% New: Compute CPA (Closest Point of Approach) between trajectory and obstacle
%% ============================================================
function [t_cpa, dist_cpa] = computeCPA(trajPos, trajVel, obsPos, obsVel)
% computeCPA Calculate Closest Point of Approach between two moving objects
%
% Description:
%   Computes the time and distance of closest approach between a trajectory
%   and an obstacle using relative motion analysis. Useful for predicting
%   potential collision risks in dynamic scenarios.
%
% Input Parameters:
%   trajPos - Trajectory position [x,y,z]
%   trajVel - Trajectory velocity [vx,vy,vz]
%   obsPos  - Obstacle position [x,y,z]
%   obsVel  - Obstacle velocity [vx,vy,vz]
%
% Output Parameters:
%   t_cpa   - Time to closest point of approach (seconds)
%   dist_cpa- Distance at closest point of approach (meters)

    % Calculate closest point of approach time and distance between two moving objects
    
    relPos = obsPos - trajPos;
    relVel = obsVel - trajVel;
    
    relVelNorm = norm(relVel);
    
    if relVelNorm < 0.01
        % Very small relative velocity, use current distance
        t_cpa = 0;
        dist_cpa = norm(relPos);
    else
        % Calculate CPA time
        t_cpa = -dot(relPos, relVel) / (relVelNorm^2);
        t_cpa = max(0, t_cpa);  % CPA cannot be in the past
        
        % Calculate CPA distance
        pos_at_cpa = relPos + relVel * t_cpa;
        dist_cpa = norm(pos_at_cpa);
    end
end