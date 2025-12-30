clear; close all; clc;

fprintf('=== 4-UAV Forced Conflict Collision Avoidance Simulation ===\n\n');
rng(42);

%% 1. Configuration Parameters
CONFIG = struct();
CONFIG.numUAVs = 4;                     % 4 UAVs
CONFIG.sim_time = 35;                   % Simulation time (seconds)
CONFIG.dt = 0.1;
CONFIG.num_steps = round(CONFIG.sim_time / CONFIG.dt);
CONFIG.speed_limit = 15;
CONFIG.acc_max = 10;
CONFIG.safety_margin = 2;               % Minimum safety distance: 2m
CONFIG.flight_altitude = 50;

% Avoidance parameters
CONFIG.lateral_offset = 5;              % Lateral offset: 5m
CONFIG.avoidance_start_dist = 40;       % Start avoidance distance: 40m
CONFIG.danger_clear_dist = 25;          % Danger clear distance: 25m
CONFIG.path_return_gain = 0.4;          % Path return gain

% Video recording settings
CONFIG.recordVideo = true;              % Enable video recording
CONFIG.videoFileName = 'multi_uav_avoidance.mp4';
CONFIG.videoFrameRate = 20;

fprintf('Configuration:\n');
fprintf('   Number of UAVs: %d\n', CONFIG.numUAVs);
fprintf('   Minimum Safety Distance: %.1fm\n', CONFIG.safety_margin);
fprintf('   Avoidance Detection Distance: %.1fm\n', CONFIG.avoidance_start_dist);
fprintf('   Path Return Distance: %.1fm\n', CONFIG.danger_clear_dist);
fprintf('   Video Recording: %s\n', string(CONFIG.recordVideo));
fprintf('   Collision Type: 4-Way Forced Conflict (Corner to Corner)\n\n');

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

%% 3. Create UAV Platforms - 4 UAVs Forced Conflict Configuration
% Flying from corners to opposite corners, must cross at center (0,0)
uav_start = [
    -70, -70, CONFIG.flight_altitude;   % UAV1: Bottom-left corner
     70, -70, CONFIG.flight_altitude;   % UAV2: Bottom-right corner
     70,  70, CONFIG.flight_altitude;   % UAV3: Top-right corner
    -70,  70, CONFIG.flight_altitude;   % UAV4: Top-left corner
];

uav_target = [
     70,  70, CONFIG.flight_altitude;   % UAV1 -> Top-right corner
    -70,  70, CONFIG.flight_altitude;   % UAV2 -> Top-left corner
    -70, -70, CONFIG.flight_altitude;   % UAV3 -> Bottom-left corner
     70, -70, CONFIG.flight_altitude;   % UAV4 -> Bottom-right corner
];

% UAV color configuration
uavColors = [
    0.0, 0.4, 0.8;   % UAV1: Blue
    0.8, 0.2, 0.2;   % UAV2: Red
    0.2, 0.7, 0.2;   % UAV3: Green
    0.9, 0.5, 0.0;   % UAV4: Orange
];

uavNames = {'UAV1 (BL->TR)', 'UAV2 (BR->TL)', 'UAV3 (TR->BL)', 'UAV4 (TL->BR)'};

fprintf('UAV Paths (All cross at center 0,0):\n');
for u = 1:CONFIG.numUAVs
    fprintf('   %s: (%.0f, %.0f) -> (%.0f, %.0f)\n', uavNames{u}, ...
        uav_start(u,1), uav_start(u,2), uav_target(u,1), uav_target(u,2));
end
fprintf('   * All 4 paths intersect at center - FORCED CONFLICT! *\n\n');

uavPlatforms = cell(CONFIG.numUAVs, 1);
for u = 1:CONFIG.numUAVs
    uavPlatforms{u} = uavPlatform("UAV" + u, scene, ...
        "InitialPosition", uav_start(u,:), ...
        "ReferenceFrame", "NED");
    initMotion = [uav_start(u,:), zeros(1,3), zeros(1,3), 1,0,0,0, zeros(1,3)];
    move(uavPlatforms{u}, initMotion);
end

%% 4-6. Create Sensors and Trackers
fprintf('Initializing sensors and trackers...\n');
lidars = cell(CONFIG.numUAVs, 1);
radars = cell(CONFIG.numUAVs, 1);
radarMountLoc = cell(CONFIG.numUAVs, 1);
radarMountAng = cell(CONFIG.numUAVs, 1);
lidarDetectors = cell(CONFIG.numUAVs, 1);
lidarTrackers = cell(CONFIG.numUAVs, 1);
radarTrackers = cell(CONFIG.numUAVs, 1);
gridTrackers = cell(CONFIG.numUAVs, 1);

for u = 1:CONFIG.numUAVs
    [lidars{u}, radars{u}, radarMountLoc{u}, radarMountAng{u}] = helperSetupSensors(uavPlatforms{u});
    [lidarDetectors{u}, lidarTrackers{u}, radarTrackers{u}, gridTrackers{u}] = ...
        helperCreateTrackers(lidars{u}, uavPlatforms{u}, radarMountLoc{u}, radarMountAng{u});
    if ~isempty(lidarDetectors{u})
        lidarDetectors{u}.Scene = scene;
    end
end

refPaths = cell(CONFIG.numUAVs, 1);
for u = 1:CONFIG.numUAVs
    waypoints = [uav_start(u,:); uav_target(u,:)];
    refPaths{u} = helperCreateReferencePath(waypoints);
end

fprintf('   Done: Sensors and trackers initialized for %d UAVs\n', CONFIG.numUAVs);

%% 7. Setup Video Recording
if CONFIG.recordVideo
    fprintf('Setting up video recording...\n');
    videoWriter = VideoWriter(CONFIG.videoFileName, 'MPEG-4');
    videoWriter.FrameRate = CONFIG.videoFrameRate;
    open(videoWriter);
    fprintf('   Done: Video writer ready: %s\n', CONFIG.videoFileName);
end

% Create animation window
figAnim = figure('Position', [50, 50, 1400, 700], 'Color', 'w', 'Name', '4-UAV Avoidance Simulation');

%% 8. Main Simulation Loop
fprintf('\nStarting simulation...\n');
fprintf('====================================================\n');

setup(scene);

stateHistory = cell(CONFIG.numUAVs, 1);
for u = 1:CONFIG.numUAVs
    stateHistory{u} = zeros(CONFIG.num_steps, 6);
end

currentPos = uav_start;
currentVel = zeros(CONFIG.numUAVs, 3);

% Initial velocity
for u = 1:CONFIG.numUAVs
    dir = uav_target(u,:) - uav_start(u,:);
    dir = dir / norm(dir);
    currentVel(u,:) = dir * CONFIG.speed_limit * 0.6;
end

% Calculate original path directions
originalDirs = zeros(CONFIG.numUAVs, 3);
for u = 1:CONFIG.numUAVs
    originalDirs(u,:) = uav_target(u,:) - uav_start(u,:);
    originalDirs(u,:) = originalDirs(u,:) / norm(originalDirs(u,:));
end

% Assign avoidance directions for 4 UAVs
% Using clockwise strategy: each UAV offsets to its right
% This ensures all UAVs avoid in the same rotation direction
avoidanceDirs = zeros(CONFIG.numUAVs, 3);
for u = 1:CONFIG.numUAVs
    % Calculate perpendicular direction to the right of flight direction
    flightDir = originalDirs(u,:);
    rightDir = cross(flightDir, [0, 0, -1]);  % Right-hand rule
    if norm(rightDir) > 0.1
        rightDir = rightDir / norm(rightDir);
    else
        rightDir = [1, 0, 0];
    end
    avoidanceDirs(u,:) = rightDir;
end

fprintf('Avoidance Strategy: All UAVs offset to their RIGHT (clockwise)\n\n');

% Distance recording - all pairs
numPairs = CONFIG.numUAVs * (CONFIG.numUAVs - 1) / 2;  % 6 pairs
pairDistances = zeros(CONFIG.num_steps, numPairs);
minDistances = zeros(CONFIG.num_steps, 1);

% Pair labels
pairLabels = cell(numPairs, 1);
pairIdx = 1;
for i = 1:CONFIG.numUAVs
    for j = i+1:CONFIG.numUAVs
        pairLabels{pairIdx} = sprintf('UAV%d-UAV%d', i, j);
        pairIdx = pairIdx + 1;
    end
end

% Statistics variables
globalMinDist = inf;
globalMinDistStep = 1;
avoidanceCount = zeros(CONFIG.numUAVs, 1);

for step = 1:CONFIG.num_steps
    time = (step - 1) * CONFIG.dt;
    
    advance(scene);
    
    % Calculate all pair distances
    pairIdx = 1;
    minDistThisStep = inf;
    for i = 1:CONFIG.numUAVs
        for j = i+1:CONFIG.numUAVs
            dist = norm(currentPos(i,:) - currentPos(j,:));
            pairDistances(step, pairIdx) = dist;
            if dist < minDistThisStep
                minDistThisStep = dist;
            end
            pairIdx = pairIdx + 1;
        end
    end
    minDistances(step) = minDistThisStep;
    
    if minDistThisStep < globalMinDist
        globalMinDist = minDistThisStep;
        globalMinDistStep = step;
    end
    
    % Control logic for each UAV
    for u = 1:CONFIG.numUAVs
        stateHistory{u}(step, :) = [currentPos(u,:), currentVel(u,:)];
        
        % Detect all other UAVs, find the closest one
        closestDist = inf;
        closestPos = [];
        for other = 1:CONFIG.numUAVs
            if other == u, continue; end
            dist = norm(currentPos(other,:) - currentPos(u,:));
            if dist < closestDist
                closestDist = dist;
                closestPos = currentPos(other,:);
            end
        end
        
        % Calculate direction and distance to target
        toTarget = uav_target(u,:) - currentPos(u,:);
        distToTarget = norm(toTarget);
        
        if distToTarget < 2
            desiredVel = [0, 0, 0];
        else
            baseDir = toTarget / distToTarget;
            baseSpeed = CONFIG.speed_limit * 0.75;
            
            % Avoidance logic
            if closestDist < CONFIG.avoidance_start_dist
                % ===== Avoidance Mode =====
                avoidanceCount(u) = avoidanceCount(u) + 1;
                
                % Calculate avoidance intensity
                if closestDist < CONFIG.safety_margin * 2
                    alpha = 1.0;
                elseif closestDist < CONFIG.safety_margin * 4
                    alpha = (CONFIG.safety_margin * 4 - closestDist) / (CONFIG.safety_margin * 2);
                else
                    alpha = 0.6 * (CONFIG.avoidance_start_dist - closestDist) / ...
                            (CONFIG.avoidance_start_dist - CONFIG.safety_margin * 4);
                end
                alpha = max(0, min(1, alpha));
                
                % Use preset avoidance direction (offset to the right)
                offsetDir = avoidanceDirs(u,:);
                
                % Also consider path return
                pathStart = uav_start(u,:);
                pathDir = originalDirs(u,:);
                toCurrentPos = currentPos(u,:) - pathStart;
                projLength = dot(toCurrentPos, pathDir);
                projPoint = pathStart + projLength * pathDir;
                returnVec = projPoint - currentPos(u,:);
                returnVec(3) = 0;
                
                if norm(returnVec) > 0.1
                    returnDir = returnVec / norm(returnVec);
                else
                    returnDir = [0, 0, 0];
                end
                
                % Adjust path return intensity based on distance
                if closestDist > CONFIG.danger_clear_dist
                    returnGain = CONFIG.path_return_gain * (closestDist - CONFIG.danger_clear_dist) / ...
                                 (CONFIG.avoidance_start_dist - CONFIG.danger_clear_dist);
                else
                    returnGain = 0;
                end
                
                % Combine velocities
                avoidanceSpeed = CONFIG.speed_limit * 0.7;
                forwardComp = baseDir * avoidanceSpeed * (1 - alpha * 0.5);
                lateralComp = offsetDir * CONFIG.lateral_offset * alpha;
                returnComp = returnDir * returnGain * norm(returnVec);
                
                desiredVel = forwardComp + lateralComp + returnComp;
                
            elseif closestDist < CONFIG.avoidance_start_dist * 1.5
                % ===== Path Return Mode =====
                pathStart = uav_start(u,:);
                pathDir = originalDirs(u,:);
                toCurrentPos = currentPos(u,:) - pathStart;
                projLength = dot(toCurrentPos, pathDir);
                projPoint = pathStart + projLength * pathDir;
                returnVec = projPoint - currentPos(u,:);
                returnVec(3) = 0;
                
                if norm(returnVec) > 0.1
                    returnDir = returnVec / norm(returnVec);
                    returnComp = returnDir * CONFIG.path_return_gain * norm(returnVec);
                else
                    returnComp = [0, 0, 0];
                end
                
                desiredVel = baseDir * baseSpeed + returnComp;
            else
                % ===== Normal Flight Mode =====
                desiredVel = baseDir * baseSpeed;
            end
        end
        
        % Speed limit
        speed = norm(desiredVel);
        if speed > CONFIG.speed_limit
            desiredVel = desiredVel / speed * CONFIG.speed_limit;
        end
        
        % Update velocity and position
        velDiff = desiredVel - currentVel(u,:);
        accNorm = norm(velDiff) / CONFIG.dt;
        if accNorm > CONFIG.acc_max
            velDiff = velDiff / accNorm * CONFIG.acc_max * CONFIG.dt;
        end
        currentVel(u,:) = currentVel(u,:) + velDiff;
        currentPos(u,:) = currentPos(u,:) + currentVel(u,:) * CONFIG.dt;
        
        % Update platform
        motion = [currentPos(u,:), currentVel(u,:), zeros(1,3), 1,0,0,0, zeros(1,3)];
        move(uavPlatforms{u}, motion);
    end
    
    % Visualization and video recording
    if mod(step, 3) == 0 || step == 1
        figure(figAnim);
        clf;
        
        % Left: 3D view
        subplot(1, 2, 1);
        hold on; grid on; axis equal;
        
        % Draw buildings
        for i = 1:size(buildings, 1)
            b = buildings(i,:);
            drawBuilding3D(b(1), b(2), b(4), b(5), b(6));
        end
        
        % Draw planned paths (dashed lines)
        for u = 1:CONFIG.numUAVs
            plot3([uav_start(u,1), uav_target(u,1)], ...
                  [uav_start(u,2), uav_target(u,2)], ...
                  [uav_start(u,3), uav_target(u,3)], ...
                  '--', 'Color', uavColors(u,:), 'LineWidth', 1.5);
        end
        
        % Draw actual trajectories
        for u = 1:CONFIG.numUAVs
            plot3(stateHistory{u}(1:step,1), ...
                  stateHistory{u}(1:step,2), ...
                  stateHistory{u}(1:step,3), ...
                  '-', 'Color', uavColors(u,:), 'LineWidth', 2.5);
        end
        
        % Draw current positions
        for u = 1:CONFIG.numUAVs
            scatter3(currentPos(u,1), currentPos(u,2), currentPos(u,3), ...
                250, uavColors(u,:), 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
            text(currentPos(u,1)+4, currentPos(u,2)+4, currentPos(u,3)+4, ...
                sprintf('UAV%d', u), 'FontSize', 11, 'FontWeight', 'bold', 'Color', uavColors(u,:));
        end
        
        % Start and end points
        scatter3(uav_start(:,1), uav_start(:,2), uav_start(:,3), ...
            120, 'g', 'filled', 'MarkerEdgeColor', 'k');
        scatter3(uav_target(:,1), uav_target(:,2), uav_target(:,3), ...
            120, 'm', 'p', 'filled', 'MarkerEdgeColor', 'k');
        
        % Center intersection point
        plot3(0, 0, CONFIG.flight_altitude, 'kx', 'MarkerSize', 20, 'LineWidth', 4);
        
        xlabel('X (m)', 'FontSize', 11);
        ylabel('Y (m)', 'FontSize', 11);
        zlabel('Z (m)', 'FontSize', 11);
        title(sprintf('4-UAV Forced Conflict | Time: %.1fs | Min Dist: %.2fm', time, minDistThisStep), ...
            'FontSize', 13);
        view(45, 35);
        xlim([-90, 90]); ylim([-90, 90]); zlim([0, 80]);
        
        % Right: Distance curves
        subplot(1, 2, 2);
        hold on; grid on;
        timeVec = (0:step-1) * CONFIG.dt;
        
        % Draw all pair distances
        pairColors = lines(numPairs);
        for p = 1:numPairs
            plot(timeVec, pairDistances(1:step, p), '-', ...
                'Color', pairColors(p,:), 'LineWidth', 1.5);
        end
        
        % Draw minimum distance (thick line)
        plot(timeVec, minDistances(1:step), 'k-', 'LineWidth', 3);
        
        % Safety distance line
        yline(CONFIG.safety_margin, 'r--', 'LineWidth', 2.5);
        
        xlabel('Time (s)', 'FontSize', 11);
        ylabel('Separation Distance (m)', 'FontSize', 11);
        title('UAV Separation Distances', 'FontSize', 13);
        legend([pairLabels; {'Minimum'}; {'Safety'}], 'Location', 'northeast', 'FontSize', 8);
        xlim([0, CONFIG.sim_time]);
        if step > 1
            ylim([0, max(max(pairDistances(1:step,:))) * 1.1]);
        end
        
        drawnow;
        
        % Write video frame
        if CONFIG.recordVideo
            frame = getframe(figAnim);
            writeVideo(videoWriter, frame);
        end
    end
    
    % Print progress
    if mod(step, 50) == 0
        fprintf('   Step %d/%d (%.1fs) - Min Distance: %.2fm\n', ...
            step, CONFIG.num_steps, time, minDistThisStep);
    end
end

%% 9. Close Video
if CONFIG.recordVideo
    close(videoWriter);
    fprintf('\nDone: Video saved: %s\n', CONFIG.videoFileName);
end

%% 10. Results Statistics
minDistTime = (globalMinDistStep - 1) * CONFIG.dt;

fprintf('\n');
fprintf('============================================================\n');
fprintf('         4-UAV FORCED CONFLICT SIMULATION RESULTS          \n');
fprintf('============================================================\n');
fprintf(' Number of UAVs:                    %d\n', CONFIG.numUAVs);
fprintf(' Simulation Time:                   %.1f s\n', CONFIG.sim_time);
fprintf(' Safety Margin:                     %.2f m\n', CONFIG.safety_margin);
fprintf('------------------------------------------------------------\n');
fprintf(' MINIMUM SEPARATION:                %.2f m\n', globalMinDist);
fprintf(' Occurred at Time:                  %.2f s\n', minDistTime);
fprintf('------------------------------------------------------------\n');

if globalMinDist >= CONFIG.safety_margin
    fprintf(' RESULT: COLLISION AVOIDANCE SUCCESSFUL!\n');
else
    fprintf(' RESULT: COLLISION RISK (%.2fm < %.2fm)\n', globalMinDist, CONFIG.safety_margin);
end

fprintf('------------------------------------------------------------\n');
fprintf(' Avoidance Maneuvers per UAV:\n');
for u = 1:CONFIG.numUAVs
    fprintf('   UAV%d: %5d steps in avoidance mode\n', u, avoidanceCount(u));
end
fprintf('============================================================\n\n');

%% 11. Save Static Images

% Figure 1: Top View
fig1 = figure('Position', [100, 100, 1000, 900], 'Color', 'w');
hold on; grid on; axis equal;

% Buildings
for i = 1:size(buildings, 1)
    b = buildings(i,:);
    rectangle('Position', [b(1)-b(4)/2, b(2)-b(5)/2, b(4), b(5)], ...
        'FaceColor', [0.7 0.7 0.75], 'EdgeColor', 'k');
end

% Planned paths and actual trajectories
for u = 1:CONFIG.numUAVs
    plot([uav_start(u,1), uav_target(u,1)], [uav_start(u,2), uav_target(u,2)], ...
        '--', 'Color', uavColors(u,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('UAV%d Planned', u));
    plot(stateHistory{u}(:,1), stateHistory{u}(:,2), ...
        '-', 'Color', uavColors(u,:), 'LineWidth', 2.5, ...
        'DisplayName', sprintf('UAV%d Actual', u));
end

% Start and end points
scatter(uav_start(:,1), uav_start(:,2), 200, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
scatter(uav_target(:,1), uav_target(:,2), 200, 'm', 'p', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Target');

% Center point
plot(0, 0, 'kx', 'MarkerSize', 25, 'LineWidth', 5, 'DisplayName', 'Intersection');

% Mark closest point
pos_min = zeros(CONFIG.numUAVs, 2);
for u = 1:CONFIG.numUAVs
    pos_min(u,:) = stateHistory{u}(globalMinDistStep, 1:2);
end
scatter(pos_min(:,1), pos_min(:,2), 100, 'k', 'filled', 'HandleVisibility', 'off');

xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title(sprintf('4-UAV Forced Conflict - Top View\nMinimum Separation: %.2fm (Safety: %.2fm)', ...
    globalMinDist, CONFIG.safety_margin), 'FontSize', 14);
legend('Location', 'bestoutside', 'FontSize', 9);
xlim([-90, 90]); ylim([-90, 90]);
saveas(fig1, 'forced_conflict_topview.png');
fprintf('Saved: forced_conflict_topview.png\n');

% Figure 2: 3D Trajectory
fig2 = figure('Position', [150, 150, 1100, 800], 'Color', 'w');
hold on; grid on; axis equal;

% Buildings
for i = 1:size(buildings, 1)
    b = buildings(i,:);
    drawBuilding3D(b(1), b(2), b(4), b(5), b(6));
end

% Trajectories
for u = 1:CONFIG.numUAVs
    plot3([uav_start(u,1), uav_target(u,1)], ...
          [uav_start(u,2), uav_target(u,2)], ...
          [uav_start(u,3), uav_target(u,3)], ...
          '--', 'Color', uavColors(u,:), 'LineWidth', 1.5);
    plot3(stateHistory{u}(:,1), stateHistory{u}(:,2), stateHistory{u}(:,3), ...
          '-', 'Color', uavColors(u,:), 'LineWidth', 3, ...
          'DisplayName', sprintf('UAV%d', u));
end

scatter3(uav_start(:,1), uav_start(:,2), uav_start(:,3), ...
    200, 'g', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
scatter3(uav_target(:,1), uav_target(:,2), uav_target(:,3), ...
    200, 'm', 'p', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Target');
plot3(0, 0, CONFIG.flight_altitude, 'kx', 'MarkerSize', 25, 'LineWidth', 5, ...
    'DisplayName', 'Intersection');

xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
zlabel('Z (m)', 'FontSize', 12);
title(sprintf('4-UAV Forced Conflict - 3D Trajectory\nMinimum Separation: %.2fm', globalMinDist), ...
    'FontSize', 14);
legend('Location', 'bestoutside', 'FontSize', 10);
view(45, 35);
saveas(fig2, 'forced_conflict_3d.png');
fprintf('Saved: forced_conflict_3d.png\n');

% Figure 3: Minimum Separation Plot (Key figure required by reviewer!)
fig3 = figure('Position', [200, 200, 1100, 600], 'Color', 'w');
hold on; grid on;
timeVec = (0:CONFIG.num_steps-1) * CONFIG.dt;

% All pair distances
pairColors = lines(numPairs);
for p = 1:numPairs
    plot(timeVec, pairDistances(:, p), '-', 'Color', pairColors(p,:), ...
        'LineWidth', 1.5, 'DisplayName', pairLabels{p});
end

% Minimum distance (thick black line)
plot(timeVec, minDistances, 'k-', 'LineWidth', 3, 'DisplayName', 'Minimum Distance');

% Safety distance line
yline(CONFIG.safety_margin, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Safety Margin');

% Mark minimum point
plot(minDistTime, globalMinDist, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', ...
    'LineWidth', 2, 'DisplayName', sprintf('Min: %.2fm @ %.1fs', globalMinDist, minDistTime));

xlabel('Time (s)', 'FontSize', 12);
ylabel('Separation Distance (m)', 'FontSize', 12);
title('4-UAV Minimum Separation Distance vs Time', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 9);
xlim([0, max(timeVec)+1]);
saveas(fig3, 'separation_distance_plot.png');
fprintf('Saved: separation_distance_plot.png\n');

fprintf('\n=== All outputs saved! ===\n');
fprintf('   Video: %s\n', CONFIG.videoFileName);
fprintf('   forced_conflict_topview.png\n');
fprintf('   forced_conflict_3d.png\n');
fprintf('   separation_distance_plot.png\n');
fprintf('\n=== Simulation Complete ===\n');

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