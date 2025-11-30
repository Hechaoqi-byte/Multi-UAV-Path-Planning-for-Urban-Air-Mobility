function candidateTrajectories = generateCandidateTrajectories(planner, currentState, refPath, speedLimit, varargin) 
    % Generate MPC candidate trajectories for motion planning
    %
    % Description:
    %   Generates multiple candidate trajectories for Model Predictive Control
    %   based on the current state and reference path. The function creates
    %   different trajectory options including speed variations and lateral
    %   offsets for obstacle avoidance. Includes special logic for formation
    %   flight scenarios to ensure coordinated avoidance maneuvers.
    %
    % Input Parameters:
    %   planner      - Motion planner object containing configuration parameters:
    %                 .timeHorizon      - Prediction time horizon (seconds)
    %                 .dt               - Time step (seconds)
    %                 .hasCloseObstacles- Boolean indicating obstacle presence
    %                 .obstaclePosition - Position of detected obstacle [x,y,z]
    %                 .numLateralOffsets- Number of lateral offset trajectories
    %                 .lateralRange     - Lateral offset range [min, max] (meters)
    %   currentState - Current UAV state vector [x,y,z,vx,vy,vz,...]
    %   refPath      - Reference path structure containing:
    %                 .xyz - Path points [N×3]
    %                 .s   - Cumulative arc length (optional)
    %   speedLimit   - Maximum allowed speed (m/s)
    %   varargin{1}  - (Optional) Distance to segment end for speed planning
    %
    % Output Parameters:
    %   candidateTrajectories - Cell array of candidate trajectories, each 
    %                          [N×14] matrix with columns:
    %                          [x,y,z, vx,vy,vz, ax,ay,az, qw,qx,qy,qz, timestamp]
    %
    % Algorithm:
    %   1. Obstacle Detection: Checks for nearby obstacles and their positions
    %   2. Trajectory Generation Modes:
    %      - No Obstacles: 3 centerline trajectories with different speeds
    %      - With Obstacles: 9 trajectories combining lateral offsets and speed variations
    %   3. Formation Flight Fix: Ensures coordinated lateral offsets for multiple UAVs
    %   4. Speed Planning: Implements acceleration-constrained speed profiles
    %
   
    % Parse distance to segment end
    distToSegmentEnd = inf; 
    if nargin >= 5 && ~isempty(varargin{1}) 
        distToSegmentEnd = varargin{1}; 
    end
    
    candidateTrajectories = {}; 
    
    if ~isstruct(refPath) || ~isfield(refPath, 'xyz') 
        warning('[MPC] refPath must be a structure'); 
        return; 
    end
    
    % Vectorized distance calculation
    currentPos = currentState(1:3); 
    distances = sum((refPath.xyz - currentPos).^2, 2);
    [~, closestIdx] = min(distances); 
    
    % Detect if there are obstacles ahead
    hasObstacle = false; 
    obstaclePos = [];
    if isfield(planner, 'hasCloseObstacles') 
        hasObstacle = planner.hasCloseObstacles; 
    end
    if isfield(planner, 'obstaclePosition')
        obstaclePos = planner.obstaclePosition;
    end
    
    % MPC parameters
    timeHorizon = planner.timeHorizon; 
    dt = planner.dt; 
    numSteps = round(timeHorizon / dt); 
    currentVel = norm(currentState(4:6)); 
    
    % Pre-allocation
    if ~hasObstacle
        candidateTrajectories = cell(3, 1);
        speedFactors = [0.8, 1.0, 1.2]; 
        numTrajectories = 3;
    else
        candidateTrajectories = cell(9, 1);
        numLateral = planner.numLateralOffsets; 
        lateralRange = planner.lateralRange; 
        lateralOffsets = linspace(lateralRange(1), lateralRange(2), numLateral); 
        speedFactors = [0.7, 1.0, 1.3]; 
        numTrajectories = 9;
    end
    
    % Lookahead distance
    lookaheadDist = max(currentVel * timeHorizon, 10); 
    
    if ~hasObstacle
        % ===== No obstacle mode: 3 centerline trajectories =====
        for i = 1:3
            [centerlinePoints, ~] = samplePathByDistance(refPath, closestIdx, lookaheadDist, numSteps); 
            traj = generateSpeedProfileTrajectory(centerlinePoints, currentState, ...
                speedLimit * speedFactors(i), planner, distToSegmentEnd); 
            candidateTrajectories{i} = traj; 
        end
        
    else
        % ===== Obstacle avoidance mode: 9 avoidance trajectories =====
        idx = 1; 
        
        [basePathPoints, tangents] = samplePathByDistance(refPath, closestIdx, lookaheadDist, numSteps); 
        
        % Calculate normal vectors (horizontal direction perpendicular to tangent)
        normals = [-tangents(:,2), tangents(:,1), zeros(size(tangents,1),1)]; 
        normNorms = sqrt(sum(normals.^2, 2)) + 1e-8;
        normals = normals ./ normNorms;
        
        % ★★★ Key fix: When dynamic obstacles detected, only generate trajectories offset to the right ★★★
        if ~isempty(obstaclePos)
            % Dynamic obstacle detected → only generate trajectories offset to the right
            % Use [0, max] instead of [-max, max] to ensure all UAVs offset to the same side
            lateralOffsets = linspace(0, abs(lateralRange(2)), numLateral);
            lateralSign = -1;  % Normals are left normal vectors, -1 means offset to the right
        else
            % When no obstacle position info, use default bidirectional trajectories
            lateralOffsets = linspace(lateralRange(1), lateralRange(2), numLateral);
            lateralSign = 1;
        end
        
        for lat = lateralOffsets
            % ★★★ Apply sign adjustment ★★★
            adjustedLat = lat * lateralSign;
            offsetPoints = basePathPoints + adjustedLat * normals; 
            
            for speedFactor = speedFactors
                traj = generateSpeedProfileTrajectory(offsetPoints, currentState, ...
                    speedLimit * speedFactor, planner, distToSegmentEnd); 
                candidateTrajectories{idx} = traj; 
                idx = idx + 1; 
            end
        end
    end
end

%% ============================================================
%% Helper Function: Sample Points Along Path
%% ============================================================
function [points, tangents] = samplePathByDistance(refPath, startIdx, totalDist, numPoints)
    points = zeros(numPoints, 3);
    tangents = zeros(numPoints, 3);
    
    pathXYZ = refPath.xyz;
    numPathPoints = size(pathXYZ, 1);
    
    if startIdx > numPathPoints
        startIdx = numPathPoints;
    end
    
    % Calculate path cumulative distance
    if isfield(refPath, 's') && ~isempty(refPath.s)
        pathS = refPath.s;
    else
        diffs = [0; sqrt(sum(diff(pathXYZ).^2, 2))];
        pathS = cumsum(diffs);
    end
    
    startS = pathS(startIdx);
    sampleS = linspace(startS, startS + totalDist, numPoints);
    
    for i = 1:numPoints
        targetS = sampleS(i);
        
        % Find corresponding path point
        idx = find(pathS >= targetS, 1, 'first');
        if isempty(idx)
            idx = numPathPoints;
        end
        
        % Interpolation
        if idx > 1 && idx <= numPathPoints
            s1 = pathS(idx-1);
            s2 = pathS(idx);
            if abs(s2 - s1) > 1e-6
                alpha = (targetS - s1) / (s2 - s1);
                alpha = max(0, min(1, alpha));
            else
                alpha = 0;
            end
            points(i,:) = (1-alpha) * pathXYZ(idx-1,:) + alpha * pathXYZ(idx,:);
        else
            points(i,:) = pathXYZ(min(idx, numPathPoints),:);
        end
        
        % Calculate tangent
        if idx < numPathPoints
            tangents(i,:) = pathXYZ(min(idx+1, numPathPoints),:) - pathXYZ(idx,:);
        elseif idx > 1
            tangents(i,:) = pathXYZ(idx,:) - pathXYZ(idx-1,:);
        else
            tangents(i,:) = [1, 0, 0];
        end
        tangentNorm = norm(tangents(i,:));
        if tangentNorm > 1e-6
            tangents(i,:) = tangents(i,:) / tangentNorm;
        end
    end
end

%% ============================================================
%% Helper Function: Generate Trajectory with Speed Planning
%% ============================================================
function trajectory = generateSpeedProfileTrajectory(waypoints, currentState, maxSpeed, planner, varargin) 
    distToSegmentEnd = inf; 
    if nargin >= 5 && ~isempty(varargin{1}) 
        distToSegmentEnd = varargin{1}; 
    end
    
    isNearSegmentEnd = (distToSegmentEnd < 20); 
    numPoints = size(waypoints, 1); 
    
    trajectory = zeros(numPoints, 14); 
    trajectory(:, 1:3) = waypoints;
    trajectory(:, 10:13) = repmat([1,0,0,0], numPoints, 1);
    
    if numPoints < 2
        return; 
    end
    
    % Calculate segment lengths
    diffs = diff(waypoints, 1, 1);
    segLengths = [0; sqrt(sum(diffs.^2, 2))];
    totalLength = sum(segLengths); 
    cumulativeLength = cumsum(segLengths); 
    
    if totalLength < 0.1
        return; 
    end
    
    % Speed planning
    currentVel = norm(currentState(4:6)); 
    startSpeed = max(currentVel, 0.1); 
    
    if isNearSegmentEnd
        endSpeed = 0.1;
    else
        endSpeed = maxSpeed * 0.9;
    end
    
    accMax = 10.0; 
    
    % Calculate segment distances
    if startSpeed < maxSpeed
        accelDist = (maxSpeed^2 - startSpeed^2) / (2 * accMax);
    else
        accelDist = 0;
        startSpeed = min(startSpeed, maxSpeed);
    end
    
    if maxSpeed > endSpeed
        decelDist = (maxSpeed^2 - endSpeed^2) / (2 * accMax);
    else
        decelDist = 0;
    end
    
    cruiseDist = totalLength - accelDist - decelDist; 
    
    if cruiseDist < 0
        maxReachableSpeed = min(sqrt((startSpeed^2 + endSpeed^2) / 2 + accMax * totalLength), maxSpeed);
        accelDist = (maxReachableSpeed^2 - startSpeed^2) / (2 * accMax); 
        decelDist = (maxReachableSpeed^2 - endSpeed^2) / (2 * accMax); 
        cruiseDist = 0; 
    end
    
    % Calculate speeds
    s = cumulativeLength;
    speeds = zeros(numPoints, 1);
    
    accelMask = s <= accelDist;
    speeds(accelMask) = sqrt(startSpeed^2 + 2 * accMax * s(accelMask));
    
    cruiseMask = (s > accelDist) & (s <= accelDist + cruiseDist);
    speeds(cruiseMask) = maxSpeed;
    
    decelMask = s > accelDist + cruiseDist;
    remainDist = totalLength - s(decelMask);
    speeds(decelMask) = sqrt(endSpeed^2 + 2 * accMax * remainDist);
    
    speeds = max(min(speeds, maxSpeed), 0.1);
    
    % Calculate velocity direction
    velDirs = [diffs; diffs(end,:)];
    normVels = sqrt(sum(velDirs.^2, 2)) + 1e-6;
    velDirs = velDirs ./ normVels;
    
    trajectory(:, 4:6) = velDirs .* speeds;
    
    % Calculate acceleration
    if numPoints > 1
        dt = planner.dt;
        trajectory(2:end, 7:9) = diff(trajectory(:, 4:6), 1, 1) / dt;
    end
    
    % Deceleration at segment end
    if isNearSegmentEnd
        finalSpeed = norm(trajectory(end, 4:6));
        if finalSpeed > 0.1
            trajectory(end, 4:6) = trajectory(end, 4:6) * 0.1 / finalSpeed;
        end
    end
end