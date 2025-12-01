%% ============================================================
%% UAV Collision Avoidance - Crossing Scenario 30° (Sensor + MPC)
%% ============================================================
% Features:
%   1. Radar sensor detection of other UAVs
%   2. MPC candidate trajectory generation
%   3. Collision validation and cost-based trajectory selection
%% ============================================================

clear; close all; clc;

fprintf('=== UAV Collision Avoidance (Crossing 30° Scenario) ===\n\n');
rng(42);

%% 1. Configuration
CONFIG = struct();
CONFIG.numUAVs = 2;
CONFIG.sim_time = 30;
CONFIG.dt = 0.1;
CONFIG.num_steps = round(CONFIG.sim_time / CONFIG.dt);
CONFIG.speed_limit = 12;
CONFIG.acc_max = 8;
CONFIG.safety_margin = 1.8;           % Safety distance: 2m
CONFIG.flight_altitude = 50;

% MPC parameters
CONFIG.timeHorizon = 2.0;           % Prediction horizon (s)
CONFIG.numLateralOffsets = 5;       % Number of lateral offsets
CONFIG.lateralRange = [-3, 3];      % Lateral offset range: ±3m
CONFIG.replanInterval = 5;          % Replan interval (steps)
CONFIG.avoidanceThreshold = 50;     % Avoidance trigger distance (m)

fprintf('MPC Parameters:\n');
fprintf('  Prediction horizon: %.1fs, Lateral offset: [%.0f, %.0f]m\n', ...
    CONFIG.timeHorizon, CONFIG.lateralRange);
fprintf('  Avoidance trigger distance: %.0fm\n\n', CONFIG.avoidanceThreshold);

%% 2. Create Scene
scene = uavScenario(...
    'ReferenceLocation', [0 0 0], ...
    'StopTime', CONFIG.sim_time, ...
    'UpdateRate', 10, ...
    'MaxNumFrames', 30);

buildings = [
    -50, -40, 0, 25, 25, 40;
     50,  40, 0, 30, 30, 45;
    -40,  50, 0, 20, 20, 35;
     40, -50, 0, 25, 25, 50;
];

for i = 1:size(buildings, 1)
    b = buildings(i, :);
    [verts, faces] = createBuildingMesh(b(1), b(2), b(4), b(5), b(6));
    addMesh(scene, "custom", {verts, faces}, [0.5 0.5 0.6]);
end

gndVerts = [-100 -100; 100 -100; 100 100; -100 100];
addMesh(scene, "polygon", {gndVerts, [0 0.1]}, [0.3 0.5 0.3]);

%% 3. Create UAV Platforms (Crossing configuration - 30 degree intersection)
% UAV1: West to East (horizontal, 0 degree)
% UAV2: Flying at 30 degree angle to UAV1
% Intersection point at (0, 0)
uav_start = [
    -70,   0, CONFIG.flight_altitude;   % UAV1: horizontal path
    -60, -35, CONFIG.flight_altitude;   % UAV2: 30 degree angle path
];
uav_target = [
     70,   0, CONFIG.flight_altitude;   % UAV1: horizontal path
     60,  35, CONFIG.flight_altitude;   % UAV2: 30 degree angle path
];

uavPlatforms = cell(CONFIG.numUAVs, 1);
for u = 1:CONFIG.numUAVs
    uavPlatforms{u} = uavPlatform("UAV" + u, scene, ...
        "InitialPosition", uav_start(u,:), ...
        "ReferenceFrame", "NED");
    initMotion = [uav_start(u,:), zeros(1,3), zeros(1,3), 1,0,0,0, zeros(1,3)];
    move(uavPlatforms{u}, initMotion);
end

%% 4. Setup Sensors
fprintf('Initializing sensors...\n');
lidars = cell(CONFIG.numUAVs, 1);
radars = cell(CONFIG.numUAVs, 1);
radarMountLoc = cell(CONFIG.numUAVs, 1);
radarMountAng = cell(CONFIG.numUAVs, 1);

for u = 1:CONFIG.numUAVs
    [lidars{u}, radars{u}, radarMountLoc{u}, radarMountAng{u}] = helperSetupSensors(uavPlatforms{u});
end
fprintf('  ✓ Sensors initialized\n');

%% 5. Create Trackers
fprintf('Initializing trackers...\n');
lidarDetectors = cell(CONFIG.numUAVs, 1);
lidarTrackers = cell(CONFIG.numUAVs, 1);
radarTrackers = cell(CONFIG.numUAVs, 1);
gridTrackers = cell(CONFIG.numUAVs, 1);

for u = 1:CONFIG.numUAVs
    [lidarDetectors{u}, lidarTrackers{u}, radarTrackers{u}, gridTrackers{u}] = ...
        helperCreateTrackers(lidars{u}, uavPlatforms{u}, radarMountLoc{u}, radarMountAng{u});
    if ~isempty(lidarDetectors{u})
        lidarDetectors{u}.Scene = scene;
    end
end
fprintf('  ✓ Trackers initialized\n');

%% 6. Create Reference Paths and Collision Validators
fprintf('Initializing MPC planners...\n');
refPaths = cell(CONFIG.numUAVs, 1);
validators = cell(CONFIG.numUAVs, 1);
planners = cell(CONFIG.numUAVs, 1);

for u = 1:CONFIG.numUAVs
    % Create reference path
    waypoints = [uav_start(u,:); uav_target(u,:)];
    refPaths{u} = helperCreateReferencePath([waypoints, zeros(2,1)]);
    
    % Create collision validator
    vehDims = struct('Length', 5, 'Width', 5, 'Height', 3);
    validators{u} = helperCreateCollisionValidator([], vehDims);
    validators{u}.safetyMargin = CONFIG.safety_margin;
    validators{u}.predictionHorizon = CONFIG.timeHorizon;
    validators{u}.predictedObstacles = {};
    
    % Initialize MPC planner
    planners{u} = struct();
    planners{u}.refPath = refPaths{u};
    planners{u}.timeHorizon = CONFIG.timeHorizon;
    planners{u}.dt = CONFIG.dt;
    planners{u}.numSpeeds = 3;
    planners{u}.numLateralOffsets = CONFIG.numLateralOffsets;
    planners{u}.lateralRange = CONFIG.lateralRange;
    planners{u}.hasCloseObstacles = false;
    planners{u}.obstaclePosition = [];
    planners{u}.uavIndex = u;
end
fprintf('  ✓ MPC planners initialized\n');

%% 7. Main Simulation Loop
fprintf('\nStarting MPC simulation...\n');
fprintf('====================================================\n');

setup(scene);

% Initialize state history
stateHistory = cell(CONFIG.numUAVs, 1);
for u = 1:CONFIG.numUAVs
    stateHistory{u} = zeros(CONFIG.num_steps, 6);
end

% Initialize current states
currentPos = uav_start;
currentVel = zeros(CONFIG.numUAVs, 3);
for u = 1:CONFIG.numUAVs
    dir = uav_target(u,:) - uav_start(u,:);
    dir = dir / norm(dir);
    currentVel(u,:) = dir * CONFIG.speed_limit * 0.6;
end

% Initialize MPC trajectory management
currentTrajs = cell(CONFIG.numUAVs, 1);
trajSteps = zeros(CONFIG.numUAVs, 1);
stepsSinceReplan = zeros(CONFIG.numUAVs, 1);

% Initialize statistics
minDistRecord = inf;
avoidCounts = zeros(CONFIG.numUAVs, 1);
sensorDetections = zeros(CONFIG.numUAVs, 1);

for step = 1:CONFIG.num_steps
    time = (step - 1) * CONFIG.dt;
    
    advance(scene);
    
    %% 7.1 Sensor Detection
    detectedTracks = cell(CONFIG.numUAVs, 1);
    
    for u = 1:CONFIG.numUAVs
        % Radar detection
        if ~isempty(radars{u})
            try
                [dets, ~, config] = detect(radars{u}, time);
                if ~isempty(dets) && ~isempty(radarTrackers{u})
                    tracks = radarTrackers{u}(dets, time);
                    if ~isempty(tracks)
                        detectedTracks{u} = tracks;
                        sensorDetections(u) = sensorDetections(u) + 1;
                    end
                end
            catch
                % Sensor detection failed, use direct position
            end
        end
        
        % If sensor didn't detect, use true position (simulating perfect sensor)
        if isempty(detectedTracks{u})
            for other = 1:CONFIG.numUAVs
                if other == u, continue; end
                % Create simulated track object
                track = struct();
                track.TrackID = other;
                track.State = [currentPos(other,1); currentVel(other,1); ...
                               currentPos(other,2); currentVel(other,2); ...
                               currentPos(other,3); currentVel(other,3)];
                detectedTracks{u} = track;
            end
        end
    end
    
    %% 7.2 Calculate UAV Distance
    dist12 = norm(currentPos(1,:) - currentPos(2,:));
    minDistRecord = min(minDistRecord, dist12);
    
    %% 7.3 Update Collision Validators
    for u = 1:CONFIG.numUAVs
        if ~isempty(detectedTracks{u})
            track = detectedTracks{u};
            if isstruct(track)
                egoState = [currentPos(u,:), currentVel(u,:)];
                try
                    validators{u} = updateCollisionValidator(validators{u}, egoState, [], track, time);
                catch
                    % Use backup update method
                    validators{u} = updateValidatorSimple(validators{u}, track, time);
                end
            end
        end
    end
    
    %% 7.4 MPC Planning
    for u = 1:CONFIG.numUAVs
        % Check if avoidance needed
        hasObstacle = dist12 < CONFIG.avoidanceThreshold;
        planners{u}.hasCloseObstacles = hasObstacle;
        
        if hasObstacle
            % Set obstacle position (other UAV)
            otherIdx = 3 - u;  % 1->2, 2->1
            planners{u}.obstaclePosition = currentPos(otherIdx,:);
        else
            planners{u}.obstaclePosition = [];
        end
        
        % Check if replanning needed
        needReplan = isempty(currentTrajs{u}) || ...
                     trajSteps(u) >= size(currentTrajs{u}, 1) || ...
                     stepsSinceReplan(u) >= CONFIG.replanInterval || ...
                     (hasObstacle && stepsSinceReplan(u) >= 2);
        
        if needReplan
            % Current state
            uavState = [currentPos(u,:), currentVel(u,:)];
            
            %% ===== MPC Candidate Trajectory Generation =====
            try
                candidateTrajs = generateCandidateTrajectories(...
                    planners{u}, uavState, refPaths{u}, CONFIG.speed_limit);
            catch ME
                fprintf('  [Warning] UAV%d generateCandidateTrajectories failed: %s\n', u, ME.message);
                candidateTrajs = {};
            end
            
            if ~isempty(candidateTrajs)
                %% ===== Kinematic Feasibility Check =====
                try
                    isFeasible = checkKinematicFeasibility(candidateTrajs, CONFIG.speed_limit, CONFIG.acc_max);
                catch
                    isFeasible = true(length(candidateTrajs), 1);
                end
                feasibleTrajs = candidateTrajs(isFeasible);
                
                if ~isempty(feasibleTrajs)
                    %% ===== Collision Validation =====
                    try
                        [isValid, collisionProb] = validateTrajectories(validators{u}, feasibleTrajs);
                    catch
                        [isValid, collisionProb] = validateTrajectoriesSimple(validators{u}, feasibleTrajs);
                    end
                    
                    % Select safe trajectories (or lowest collision probability)
                    if any(isValid)
                        safeIdx = find(isValid);
                    else
                        [~, sortIdx] = sort(collisionProb);
                        safeIdx = sortIdx(1:min(3, length(sortIdx)));
                    end
                    
                    safeTrajs = feasibleTrajs(safeIdx);
                    safeProbs = collisionProb(safeIdx);
                    
                    %% ===== Cost Function Calculation =====
                    try
                        costs = calculateTrajectoryCosts(safeTrajs, safeProbs, uavState, refPaths{u});
                    catch
                        costs = calculateCostsSimple(safeTrajs, safeProbs, uavState, refPaths{u});
                    end
                    
                    %% ===== Select Optimal Trajectory =====
                    [~, optIdx] = min(costs);
                    bestTraj = safeTrajs{optIdx};
                    
                    % Statistics
                    if hasObstacle && (optIdx > 1 || ~any(isValid))
                        avoidCounts(u) = avoidCounts(u) + 1;
                    end
                    
                    % Store trajectory
                    if hasObstacle
                        controlSteps = min(15, size(bestTraj, 1));  % Execute more steps during avoidance
                    else
                        controlSteps = min(CONFIG.replanInterval + 2, size(bestTraj, 1));
                    end
                    currentTrajs{u} = bestTraj(1:controlSteps, :);
                    trajSteps(u) = 1;
                    stepsSinceReplan(u) = 0;
                    
                    % Debug output
                    if hasObstacle && mod(step, 20) == 0
                        fprintf('  [UAV%d] MPC selected trajectory %d/%d, collision prob=%.2f\n', ...
                            u, optIdx, length(safeTrajs), safeProbs(optIdx));
                    end
                end
            end
        end
        
        %% 7.5 Execute Trajectory
        if ~isempty(currentTrajs{u}) && trajSteps(u) < size(currentTrajs{u}, 1)
            trajSteps(u) = trajSteps(u) + 1;
            stepsSinceReplan(u) = stepsSinceReplan(u) + 1;
            
            nextState = currentTrajs{u}(trajSteps(u), :);
            currentPos(u,:) = nextState(1:3);
            if size(nextState, 2) >= 6
                currentVel(u,:) = nextState(4:6);
            end
        else
            % Backup: simple fly towards target
            toTarget = uav_target(u,:) - currentPos(u,:);
            if norm(toTarget) > 1
                dir = toTarget / norm(toTarget);
                currentVel(u,:) = dir * CONFIG.speed_limit * 0.8;
                currentPos(u,:) = currentPos(u,:) + currentVel(u,:) * CONFIG.dt;
            end
        end
        
        % Maintain altitude
        currentPos(u,3) = CONFIG.flight_altitude;
        
        % Record history
        stateHistory{u}(step, :) = [currentPos(u,:), currentVel(u,:)];
        
        % Update platform
        motion = [currentPos(u,:), currentVel(u,:), zeros(1,3), 1,0,0,0, zeros(1,3)];
        move(uavPlatforms{u}, motion);
    end
    
    %% 7.6 Print Progress
    if mod(step, 50) == 0 || (dist12 < CONFIG.avoidanceThreshold && mod(step, 20) == 0)
        status = '';
        if dist12 < CONFIG.avoidanceThreshold
            status = ' [MPC Avoidance]';
        end
        fprintf('t=%5.1fs | dist=%5.1fm | UAV1:(%.0f,%.0f) UAV2:(%.0f,%.0f)%s\n', ...
            time, dist12, currentPos(1,1), currentPos(1,2), ...
            currentPos(2,1), currentPos(2,2), status);
    end
    
    %% 7.7 Termination Condition
    if norm(currentPos(1,:) - uav_target(1,:)) < 5 && ...
       norm(currentPos(2,:) - uav_target(2,:)) < 5
        fprintf('\n✓ Both UAVs reached their targets!\n');
        break;
    end
end

%% 8. Results Analysis
fprintf('\n====================================================\n');
fprintf('              Simulation Results\n');
fprintf('====================================================\n');

% Calculate distance curve
distances = zeros(step, 1);
for s = 1:step
    distances(s) = norm(stateHistory{1}(s,1:3) - stateHistory{2}(s,1:3));
end
[minDist, minStep] = min(distances);

fprintf('  Simulation time: %.1f s\n', time);
fprintf('  Minimum distance: %.2f m\n', minDist);
fprintf('  Safety margin: %.1f m\n', CONFIG.safety_margin);
fprintf('  UAV1 MPC avoidance count: %d\n', avoidCounts(1));
fprintf('  UAV2 MPC avoidance count: %d\n', avoidCounts(2));
fprintf('  Sensor detections: UAV1=%d, UAV2=%d\n', sensorDetections(1), sensorDetections(2));

if minDist >= CONFIG.safety_margin
    fprintf('  Result: ✓ Avoidance successful!\n');
else
    fprintf('  Result: ⚠ Collision risk!\n');
end
fprintf('====================================================\n');

%% 9. Visualization - Save to 3 separate figures

% Figure 1: Top View
fig1 = figure('Position', [100, 100, 800, 700]);
hold on; grid on; axis equal;
for i = 1:size(buildings, 1)
    b = buildings(i,:);
    rectangle('Position', [b(1)-b(4)/2, b(2)-b(5)/2, b(4), b(5)], ...
        'FaceColor', [0.7 0.7 0.75], 'EdgeColor', 'k');
end
plot([uav_start(1,1), uav_target(1,1)], [uav_start(1,2), uav_target(1,2)], ...
    'b--', 'LineWidth', 1.5, 'DisplayName', 'UAV1 Planned Path');
plot([uav_start(2,1), uav_target(2,1)], [uav_start(2,2), uav_target(2,2)], ...
    'r--', 'LineWidth', 1.5, 'DisplayName', 'UAV2 Planned Path');
plot(stateHistory{1}(1:step,1), stateHistory{1}(1:step,2), ...
    'b-', 'LineWidth', 2.5, 'DisplayName', 'UAV1 Actual Path');
plot(stateHistory{2}(1:step,1), stateHistory{2}(1:step,2), ...
    'r-', 'LineWidth', 2.5, 'DisplayName', 'UAV2 Actual Path');
scatter(uav_start(:,1), uav_start(:,2), 150, 'g', 'filled', 'DisplayName', 'Start');
scatter(uav_target(:,1), uav_target(:,2), 150, 'm', 'p', 'filled', 'DisplayName', 'Target');
plot(0, 0, 'kx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Intersection');
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12);
title(sprintf('Crossing 30° Scenario - Top View (Min Distance: %.2fm)', minDist), 'FontSize', 14);
legend('Location', 'bestoutside', 'FontSize', 10);
xlim([-90, 90]); ylim([-90, 90]);
saveas(fig1, 'crossing30_topview.png');
fprintf('Saved: crossing30_topview.png\n');

% Figure 2: 3D Trajectory
fig2 = figure('Position', [150, 150, 900, 700]);
hold on; grid on; axis equal;
for i = 1:size(buildings, 1)
    b = buildings(i,:);
    drawBuilding3D(b(1), b(2), b(4), b(5), b(6));
end
plot3([uav_start(1,1), uav_target(1,1)], [uav_start(1,2), uav_target(1,2)], ...
    [uav_start(1,3), uav_target(1,3)], 'b--', 'LineWidth', 1.5, 'DisplayName', 'UAV1 Planned');
plot3([uav_start(2,1), uav_target(2,1)], [uav_start(2,2), uav_target(2,2)], ...
    [uav_start(2,3), uav_target(2,3)], 'r--', 'LineWidth', 1.5, 'DisplayName', 'UAV2 Planned');
plot3(stateHistory{1}(1:step,1), stateHistory{1}(1:step,2), stateHistory{1}(1:step,3), ...
    'b-', 'LineWidth', 2.5, 'DisplayName', 'UAV1 Actual');
plot3(stateHistory{2}(1:step,1), stateHistory{2}(1:step,2), stateHistory{2}(1:step,3), ...
    'r-', 'LineWidth', 2.5, 'DisplayName', 'UAV2 Actual');
scatter3(uav_start(:,1), uav_start(:,2), uav_start(:,3), 150, 'g', 'filled', 'DisplayName', 'Start');
scatter3(uav_target(:,1), uav_target(:,2), uav_target(:,3), 150, 'm', 'p', 'filled', 'DisplayName', 'Target');
plot3(0, 0, CONFIG.flight_altitude, 'kx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Intersection');
xlabel('X (m)', 'FontSize', 12); ylabel('Y (m)', 'FontSize', 12); zlabel('Z (m)', 'FontSize', 12);
title(sprintf('Crossing 30° Scenario - 3D Trajectory (Min Distance: %.2fm)', minDist), 'FontSize', 14);
legend('Location', 'bestoutside', 'FontSize', 10);
view(45, 30);
saveas(fig2, 'crossing30_3d.png');
fprintf('Saved: crossing30_3d.png\n');

% Figure 3: Distance vs Time
fig3 = figure('Position', [200, 200, 800, 500]);
timeVec = (0:step-1) * CONFIG.dt;
plot(timeVec, distances, 'b-', 'LineWidth', 2, 'DisplayName', 'UAV Distance');
hold on; grid on;
yline(CONFIG.safety_margin, 'r--', 'LineWidth', 2, 'DisplayName', 'Safety Margin');
plot(timeVec(minStep), minDist, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', ...
    'DisplayName', sprintf('Minimum: %.2fm', minDist));
xlabel('Time (s)', 'FontSize', 12); ylabel('Distance (m)', 'FontSize', 12);
title('Crossing 30° Scenario - UAV Separation Distance', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 11);
xlim([0, max(timeVec)+1]);
saveas(fig3, 'crossing30_distance.png');
fprintf('Saved: crossing30_distance.png\n');

fprintf('\nAll figures saved!\n');

%% ============================================================
%% Helper Functions
%% ============================================================

function [verts, faces] = createBuildingMesh(cx, cy, sx, sy, sz)
    dx = sx/2; dy = sy/2;
    verts = [cx-dx,cy-dy,0; cx+dx,cy-dy,0; cx+dx,cy+dy,0; cx-dx,cy+dy,0;
             cx-dx,cy-dy,sz; cx+dx,cy-dy,sz; cx+dx,cy+dy,sz; cx-dx,cy+dy,sz];
    faces = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8];
end

function drawBuilding3D(cx, cy, sx, sy, sz)
    dx = sx/2; dy = sy/2;
    x = cx + dx * [-1 1 1 -1 -1 1 1 -1];
    y = cy + dy * [-1 -1 1 1 -1 -1 1 1];
    z = [0 0 0 0 sz sz sz sz];
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    for i = 1:6
        f = faces(i,:);
        fill3(x(f), y(f), z(f), [0.6 0.65 0.7], 'FaceAlpha', 0.4, 'EdgeColor', 'k');
    end
end

function validator = updateValidatorSimple(validator, track, time)
    % Simple validator update
    if time - validator.lastUpdateTime < 0.05
        return;
    end
    
    validator.predictedObstacles = {};
    
    if isempty(track)
        validator.lastUpdateTime = time;
        return;
    end
    
    T = validator.predictionHorizon;
    dt = 0.1;
    numSteps = round(T / dt) + 1;
    
    state = track.State;
    pos = [state(1); state(3); state(5)];
    vel = [state(2); state(4); state(6)];
    
    futurePos = zeros(numSteps, 3);
    for t = 1:numSteps
        futurePos(t, :) = (pos + vel * (t-1) * dt)';
    end
    
    validator.predictedObstacles{1} = futurePos;
    validator.lastUpdateTime = time;
end

function [isValid, collisionProb] = validateTrajectoriesSimple(validator, trajectories)
    % Simple collision validation
    numTrajs = length(trajectories);
    isValid = true(numTrajs, 1);
    collisionProb = zeros(numTrajs, 1);
    
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
        
        for j = 1:length(validator.predictedObstacles)
            obsPred = validator.predictedObstacles{j};
            if isempty(obsPred), continue; end
            
            numSteps = min(size(trajPos, 1), size(obsPred, 1));
            for t = 1:numSteps
                dist = norm(trajPos(t, :) - obsPred(t, :));
                minDist = min(minDist, dist);
            end
        end
        
        if minDist < safetyMargin * 0.5
            isValid(i) = false;
            collisionProb(i) = 1.0;
        elseif minDist < safetyMargin
            isValid(i) = false;
            collisionProb(i) = 0.5 + 0.5 * (1 - minDist / safetyMargin);
        elseif minDist < safetyMargin * 1.5
            isValid(i) = true;
            collisionProb(i) = 0.3 * (1 - (minDist - safetyMargin) / (safetyMargin * 0.5));
        else
            isValid(i) = true;
            collisionProb(i) = 0;
        end
    end
end

function costs = calculateCostsSimple(trajectories, collisionProb, currentState, refPath)
    % Simple cost calculation
    costs = zeros(length(trajectories), 1);
    
    for i = 1:length(trajectories)
        traj = trajectories{i};
        if isempty(traj) || size(traj, 1) < 2
            costs(i) = 1e4;
            continue;
        end
        
        % Collision cost
        collisionCost = collisionProb(i) * 1000;
        
        % Progress cost (negative means forward progress)
        startPos = currentState(1:3);
        endPos = traj(end, 1:3);
        distStart = vecnorm(refPath.xyz - startPos, 2, 2);
        [~, idxStart] = min(distStart);
        distEnd = vecnorm(refPath.xyz - endPos, 2, 2);
        [~, idxEnd] = min(distEnd);
        progressCost = -(idxEnd - idxStart) * 5;
        
        % Path deviation cost
        trajPos = traj(:, 1:3);
        pathDev = 0;
        for j = 1:size(trajPos, 1)
            dists = vecnorm(refPath.xyz - trajPos(j,:), 2, 2);
            pathDev = pathDev + min(dists);
        end
        pathCost = pathDev / size(trajPos, 1) * 2;
        
        costs(i) = collisionCost + progressCost + pathCost;
    end
end