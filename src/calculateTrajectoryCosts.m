function costs = calculateTrajectoryCosts(trajectories, collisionProb, currentState, refPath, varargin) 
% calculateTrajectoryCosts Calculate comprehensive cost for multiple trajectories
%
% Description:
%   Evaluates and scores multiple trajectories based on various criteria including
%   collision probability, path following, progress, smoothness, and continuity.
%   Used for trajectory selection in motion planning systems.
%
% Input Parameters:
%   trajectories      - Cell array of trajectories, each [N×6] with [x,y,z,vx,vy,vz]
%   collisionProb     - Vector of collision probabilities for each trajectory
%   currentState      - Current state vector [x,y,z,vx,vy,vz] of the vehicle
%   refPath           - Reference path structure with fields:
%                       .xyz     - Path points [M×3]
%                       .s       - Arc length parameters (optional)
%                       .tangents - Tangent vectors (optional)
%   varargin          - Optional parameters in order:
%                       {1} previousTrajectory - Previous optimal trajectory [K×6]
%                       {2} stopPoints         - Stop point locations [P×3]
%                       {3} stopRadius         - Stop radius threshold (default: 15.0)
%                       {4} minAllowedS        - Minimum allowed arc length
%
% Output Parameters:
%   costs             - Cost vector for each trajectory, lower is better
%


    costs = zeros(length(trajectories), 1); 
    
    % -------- Parse optional parameters -------- 
    previousTrajectory = []; 
    if nargin >= 5
        previousTrajectory = varargin{1}; 
    end
    
    stopPoints = []; 
    if nargin >= 6
        stopPoints = varargin{2}; 
    end
    
    stopRadius = 0; 
    if nargin >= 7
        stopRadius = varargin{3}; 
    end
    if isempty(stopRadius) || stopRadius <= 0
        stopRadius = 15.0;
    end
    
    % Minimum allowed arc length
    minAllowedS = -inf;
    if nargin >= 8 && ~isempty(varargin{4})
        minAllowedS = varargin{4};
    end
    
    hasRefS = isfield(refPath, 's') && ~isempty(refPath.s); 
    
    % ★★★ Key fix: Significantly increase collision weight ★★★
    collisionThreshold = 0.05;    % Lower threshold (original 0.10): more sensitive collision detection
    collisionWeight    = 2000;    % Significantly increase weight (original 150): higher cost for collision trajectories
    
    for i = 1:length(trajectories) 
        traj = trajectories{i}; 
        if isempty(traj) || size(traj,1) < 2
            costs(i) = 1e4; 
            continue; 
        end
        
        %% 1. Collision cost - ★★★ Fix: Stricter collision penalty ★★★
        if collisionProb(i) > collisionThreshold
            % Directly reject when collision probability is high
            costs(i) = 1e4 + collisionProb(i) * 1e4;  % Higher probability = higher cost
            continue; 
        end
        
        % Penalty even for low probabilities
        collisionCost = collisionProb(i) * collisionWeight; 
        
        % ★★★ New: Additional penalty based on collision probability (non-linear) ★★★
        if collisionProb(i) > 0.01
            collisionCost = collisionCost + 500 * (collisionProb(i)^0.5);
        end
        
        %% 2. Forward progress cost
        startPos = currentState(1:3); 
        endPos   = traj(end, 1:3); 
        progressS = 0; 
        
        if hasRefS
            % Current point arc length
            distsCurr = vecnorm(refPath.xyz - startPos, 2, 2); 
            [~, idxCurr] = min(distsCurr); 
            s_curr = refPath.s(idxCurr); 
            
            % End point arc length
            distsEnd = vecnorm(refPath.xyz - endPos, 2, 2); 
            [~, idxEnd] = min(distsEnd); 
            s_end = refPath.s(idxEnd); 
            
            % Check if retreating to already traversed area
            if s_end < (minAllowedS - 2.0)
                costs(i) = 1e4;
                continue;
            end
            
            % Calculate progress
            progressS = s_end - s_curr;
            
            % Check if current step is retreating
            if progressS < -1.0
                costs(i) = costs(i) + 5000;
                progressS = 0;
            end
        else
            % No arc length information: project to tangent
            if isfield(refPath, 'tangents') && ~isempty(refPath.tangents) 
                avgTangent = mean(refPath.tangents, 1); 
                avgTangent = avgTangent / (norm(avgTangent) + 1e-6); 
                displacement = endPos - startPos; 
                progressS = dot(displacement, avgTangent);
                
                if progressS < -1.0
                    costs(i) = costs(i) + 5000;
                    progressS = 0;
                end
            else
                progressS = norm(endPos - startPos); 
            end
        end
        
        % Progress reward
        k_prog_s = 8.0;
        progressCost = -k_prog_s * max(0, progressS);
        
        %% 3. Path deviation cost - ★★★ Fix: Reduce weight, allow larger deviation ★★★
        trajPos = traj(:,1:3); 
        pathDeviations = zeros(size(trajPos,1),1); 
        for j = 1:size(trajPos,1) 
            distances = vecnorm(refPath.xyz - trajPos(j,:), 2, 2); 
            pathDeviations(j) = min(distances); 
        end
        pathCost = mean(pathDeviations); 
        
        % ★★★ Fix: Reduce path deviation penalty when collision risk exists ★★★
        if collisionProb(i) > 0.01
            pathCostWeight = 1.0;  % Reduce deviation penalty when collision risk exists
        else
            pathCostWeight = 2.0;  % Normal penalty when no collision risk
        end
        
        %% 4. Speed cost
        speeds   = vecnorm(traj(:,4:6), 2, 2); 
        avgSpeed = mean(speeds); 
        targetSpeed   = 15.0;
        speedCost_avg = (targetSpeed - avgSpeed)^2 * 0.008; 
        
        endVel   = traj(end,4:6); 
        endSpeed = norm(endVel); 
        
        isNearStop = false; 
        if ~isempty(stopPoints) 
            distsToStops = vecnorm(stopPoints - endPos, 2, 2); 
            minStopDist  = min(distsToStops); 
            if minStopDist < stopRadius * 2.0
                isNearStop = true; 
            end
        end
        
        if isNearStop
            desiredEndSpeed = 0.3; 
            speedCost_end = max(0, endSpeed - desiredEndSpeed)^2 * 0.05; 
        else
            slowDownWeight = 0.5;
            speedCost_end  = slowDownWeight * (endSpeed^2) * 0.005; 
        end
        speedCost = speedCost_avg + speedCost_end; 
        
        %% 5. Smoothness cost
        if size(traj,1) > 3
            velocities    = traj(:,4:6); 
            accelerations = diff(velocities,1,1)/0.1; 
            jerks         = diff(accelerations,1,1)/0.1; 
            jerkMag       = vecnorm(jerks,2,2); 
            jerkCost      = mean(jerkMag.^2) * 0.01;
        else
            jerkCost = 0; 
        end
        
        %% 6. Continuity cost
        continuityCost = 0; 
        if ~isempty(previousTrajectory) && size(previousTrajectory,1) > 0
            prevEndPos   = previousTrajectory(end,1:3); 
            currStartPos = traj(1,1:3); 
            posDiff      = norm(currStartPos - prevEndPos); 
            continuityCost = continuityCost + posDiff * 1.0;
            
            prevEndVel   = previousTrajectory(end,4:6); 
            currStartVel = traj(1,4:6); 
            velDiff      = norm(currStartVel - prevEndVel); 
            continuityCost = continuityCost + velDiff * 0.3;
        end
        
        %% 7. Total cost - ★★★ Fix: Collision cost has highest priority ★★★
        costs(i) = progressCost + ... 
                   pathCostWeight * pathCost + ...  % Dynamic weight
                   speedCost + ... 
                   jerkCost + ... 
                   collisionCost + ...              % Significantly increased collision cost
                   continuityCost; 
    end
end