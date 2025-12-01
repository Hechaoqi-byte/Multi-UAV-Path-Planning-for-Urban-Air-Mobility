function [isValid, collisionProb] = validateTrajectories(validator, trajectories)
% validateTrajectories Validate trajectories against predicted obstacles with collision probability
%
% Description:
%   Evaluates multiple candidate trajectories for potential collisions with
%   predicted obstacle positions. Computes minimum distances to obstacles
%   and converts them into collision probabilities using a multi-threshold
%   approach. Includes intermediate point interpolation for more accurate
%   collision detection between discrete time steps.
%
% Input Parameters:
%   validator    - Collision validator structure containing:
%                  .predictedObstacles - Cell array of predicted obstacle trajectories
%                  .safetyMargin       - Safety distance threshold (meters)
%   trajectories - Cell array of candidate trajectories, each [N×M] where:
%                  First 3 columns are position coordinates [x,y,z]
%
% Output Parameters:
%   isValid      - Logical vector [numTrajs×1] indicating valid trajectories
%                  true: trajectory is safe, false: trajectory has collision risk
%   collisionProb- Probability vector [numTrajs×1] ranging [0,1] where:
%                  0: no collision risk, 1: certain collision
%
% Algorithm:
%   1. Input validation and parameter initialization
%   2. For each trajectory, compute minimum distance to all predicted obstacles
%   3. Include intermediate point interpolation for continuous collision checking
%   4. Convert minimum distances to collision probabilities using safety margins
%   5. Apply multi-level probability thresholds for risk assessment
%
% Collision Probability Mapping:
%   - Distance < 0.3×safetyMargin: 100% collision probability
%   - 0.3×safetyMargin ≤ Distance < 0.6×safetyMargin: 80-100% probability  
%   - 0.6×safetyMargin ≤ Distance < safetyMargin: 40-80% probability
%   - safetyMargin ≤ Distance < 1.5×safetyMargin: 15-40% probability
%   - 1.5×safetyMargin ≤ Distance < 2.0×safetyMargin: 5-15% probability
%   - Distance ≥ 2.0×safetyMargin: 0% collision probability
%
% Features:
%   - Multi-threshold collision probability calculation
%   - Intermediate point interpolation for accurate collision detection
%   - Robust handling of empty inputs and edge cases
%   - Configurable safety margins

    % Validate trajectories for collisions with obstacles
    
    numTrajs = length(trajectories);
    isValid = true(numTrajs, 1);
    collisionProb = zeros(numTrajs, 1);
    
    % 1. Check if obstacle predictions are available
    if ~isfield(validator, 'predictedObstacles') || isempty(validator.predictedObstacles)
        return;
    end
    
    % 2. Get safety margin parameter
    if isfield(validator, 'safetyMargin')
        safetyMargin = validator.safetyMargin;
    else
        safetyMargin = 10;
    end
    
    % 3. Validate each trajectory
    for i = 1:numTrajs
        traj = trajectories{i};
        
        % 3.1 Check trajectory validity
        if isempty(traj) || size(traj, 1) < 2
            isValid(i) = false;
            collisionProb(i) = 1.0;
            continue;
        end
        
        trajPos = traj(:, 1:3);
        minDist = inf;
        
        % 3.2 Check against all predicted obstacles
        for j = 1:length(validator.predictedObstacles)
            obsPred = validator.predictedObstacles{j};
            
            if isempty(obsPred)
                continue;
            end
            
            % 3.2.1 Check discrete time steps
            numSteps = min(size(trajPos, 1), size(obsPred, 1));
            
            for t = 1:numSteps
                dist = norm(trajPos(t, :) - obsPred(t, :));
                minDist = min(minDist, dist);
            end
            
            % 3.2.2 Check intermediate points for continuous collision detection
            if numSteps > 1
                for t = 1:numSteps-1
                    for alpha = 0.25:0.25:0.75
                        % Interpolate trajectory point
                        interpTraj = trajPos(t,:) * (1-alpha) + trajPos(t+1,:) * alpha;
                        % Interpolate obstacle position
                        t2 = min(t+1, size(obsPred,1));
                        interpObs = obsPred(t,:) * (1-alpha) + obsPred(t2,:) * alpha;
                        % Calculate distance
                        dist = norm(interpTraj - interpObs);
                        minDist = min(minDist, dist);
                    end
                end
            end
        end
        
        % 3.3 Convert minimum distance to collision probability
        if minDist < safetyMargin * 0.3
            % Very high risk: certain collision
            isValid(i) = false;
            collisionProb(i) = 1.0;
        elseif minDist < safetyMargin * 0.6
            % High risk: 80-100% collision probability
            isValid(i) = false;
            collisionProb(i) = 0.8 + 0.2 * (1 - (minDist - safetyMargin*0.3) / (safetyMargin*0.3));
        elseif minDist < safetyMargin
            % Medium risk: 40-80% collision probability
            isValid(i) = false;
            collisionProb(i) = 0.4 + 0.4 * (1 - (minDist - safetyMargin*0.6) / (safetyMargin*0.4));
        elseif minDist < safetyMargin * 1.5
            % Low risk: 15-40% collision probability
            isValid(i) = true;
            collisionProb(i) = 0.15 + 0.25 * (1 - (minDist - safetyMargin) / (safetyMargin*0.5));
        elseif minDist < safetyMargin * 2.0
            % Very low risk: 5-15% collision probability
            isValid(i) = true;
            collisionProb(i) = 0.05 + 0.1 * (1 - (minDist - safetyMargin*1.5) / (safetyMargin*0.5));
        else
            % No risk: safe trajectory
            isValid(i) = true;
            collisionProb(i) = 0;
        end
    end
end