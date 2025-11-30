clear; close all; clc;

fprintf('=== Manhattan Multi-UAV System (Task Allocation + BiRRT + Dynamic Obstacle Avoidance) ===\n\n');

rng(2021);
%% ============================================================
%% 0. System Configuration Area (Modify parameters here)
%% ============================================================

fprintf('====================================================\n');
fprintf('System Configuration\n');
fprintf('====================================================\n\n');

% ===== Core Configuration Parameters =====
CONFIG = struct();

% UAV and Charging Station Configuration
CONFIG.numUAVs = 9;
CONFIG.numChargers = 9;
CONFIG.charger_mode = 'auto';        % Charging station layout: 'auto', 'corners', 'grid', 'circle'
CONFIG.charger_height = 180;         % Charging station height (meters)

% Task Point Configuration
CONFIG.numTasks = 9;                % <- Number of task points (recommended = numUAVs × 2~3)
CONFIG.task_mode = 'random';         % Task point generation: 'random', 'custom'
CONFIG.task_z_min = 160;             % Task point minimum height (meters)
CONFIG.task_z_max = 200;             % Task point maximum height (meters)

% MTSP Optimization Configuration
CONFIG.mtsp_q1 = 0.05;               % Charging station weight (low -> encourage more UAVs)
CONFIG.mtsp_q2 = 0.55;               % Distance weight (high -> energy saving)
CONFIG.mtsp_q3 = 0.40;               % Time weight (high -> fast)
CONFIG.distance_metric = 'manhattan_obstacles';  % Distance metric
CONFIG.vertical_penalty = 1.3;       % Vertical flight penalty

% GA Parameters
CONFIG.ga_population = 60;           % GA population size
CONFIG.ga_generations = 100;         % GA generations

% Simulation Parameters
CONFIG.sim_time = 1000;              % Simulation steps (total duration = sim_time * 0.1 s)
CONFIG.speed_limit = 15;             % Speed limit (m/s)
CONFIG.acc_max = 50;                 % Acceleration limit (m/s²)

% ===== Minimum Snap Configuration =====
CONFIG.minimum_snap.enabled = true;              % Enable Minimum Snap
CONFIG.minimum_snap.corridor_enabled = false;    % Enable corridor constraints  
CONFIG.minimum_snap.corridor_width = 8.0;        % Corridor width (meters)
CONFIG.minimum_snap.max_waypoints = 10;          % Maximum key points
CONFIG.minimum_snap.n_intermediate = 4;          % Minimum constraint points

% ===== Memory Configuration Parameters =====
MEMORY_CONFIG = struct();
% Dynamically set history record size (based on simulation steps, with margin)
MEMORY_CONFIG.maxHistorySteps = max(1000, CONFIG.sim_time * 2);

fprintf('Configuration Overview:\n');
fprintf('  Number of UAVs: %d\n', CONFIG.numUAVs);
fprintf('  Number of Charging Stations: %d\n', CONFIG.numChargers); 
fprintf('  Charging Station Layout: %s\n', CONFIG.charger_mode);
fprintf('  Number of Task Points: %d\n', CONFIG.numTasks);
fprintf('  Task Height: %.0f-%.0fm\n', CONFIG.task_z_min, CONFIG.task_z_max);
fprintf('  Distance Metric: %s\n', CONFIG.distance_metric);
fprintf('  Optimization Objective: Energy Saving(%.0f%%) + Speed(%.0f%%)\n', ...
    CONFIG.mtsp_q2*100, CONFIG.mtsp_q3*100);
fprintf('  Simulation Duration: %.1f seconds\n', CONFIG.sim_time * 0.1);
fprintf('  Max History Records: %d steps\n', MEMORY_CONFIG.maxHistorySteps);

fprintf('====================================================\n\n');

%% ============================================================
%% 1. Create Scene (Supporting Multiple UAVs)
%% ============================================================
fprintf('1. Creating scene...\n');

% Calculate total simulation time
simStopTime = CONFIG.sim_time * 0.1;  % Convert steps to seconds

% Calculate required maximum coordinate frames
% Each UAV needs about 3-4 frames (platform + sensors)
requiredFrames = CONFIG.numUAVs * 5 + 20;  % Reserve 20 extra frames

fprintf('  Configuration Parameters:\n');
fprintf('    Number of UAVs: %d\n', CONFIG.numUAVs);
fprintf('    Simulation Duration: %.1f seconds (%d steps)\n', simStopTime, CONFIG.sim_time);
fprintf('    Required Coordinate Frames: %d\n', requiredFrames);

% Create scene (Key: set large enough MaxNumFrames)
scene = uavScenario(...
    'ReferenceLocation', [40.707088 -74.012146 0], ...  % Manhattan coordinates
    'StopTime', simStopTime, ...                       % Simulation duration
    'UpdateRate', 5, ...                               % 5 Hz
    'MaxNumFrames', max(100, requiredFrames));         % Ensure sufficient frames

fprintf('  ✓ Scene created successfully\n');
fprintf('    MaxNumFrames: %d\n', max(100, requiredFrames));
fprintf('    UpdateRate: 5 Hz\n');
fprintf('    ReferenceLocation: Manhattan\n\n');

%% 2. Load Terrain Data
fprintf('2. Loading terrain data...\n');
xlimits_terrain = [-550 550];
ylimits_terrain = [-550 550];
terrain_color    = [0.6 0.6 0.6];
try
    addMesh(scene, "terrain", {"gmted2010", xlimits_terrain, ylimits_terrain}, ...
        terrain_color, Verbose=false);
    fprintf('  ✓ Terrain loaded successfully\n');
catch
    fprintf('  ⚠ Terrain loading failed\n');
end

%% 3. Load OSM Building Data
fprintf('3. Loading OSM building data...\n');
xlimits_osm    = [-500 500];
ylimits_osm    = [-500 500];
building_color = [0 1 0];
try
    addMesh(scene, "buildings", {"manhattan.osm", xlimits_osm, ylimits_osm, "auto"}, ...
        building_color, Verbose=false);
    fprintf('  ✓ Buildings loaded successfully\n');
catch
    fprintf('  ⚠ Building loading failed\n');
end

%% 4. Get Scene Meshes
fprintf('4. Getting scene meshes...\n');
allMeshes = scene.Meshes;
fprintf('  Total meshes in scene: %d\n', length(allMeshes));

if isempty(allMeshes)
    error('No meshes loaded in the scene!');
end

%% 5. Filter Building Meshes
fprintf('5. Filtering building meshes...\n');
osmXRange = [-500 500];
osmYRange = [-500 500];
minBuildingHeight = 5;
buildingMeshes = [];

for i = 1:length(allMeshes)
    currentMesh = allMeshes{i};
    verts = currentMesh.Vertices;
    if isempty(verts)
        continue;
    end

    xInRange = all(verts(:,1) >= osmXRange(1)) && all(verts(:,1) <= osmXRange(2));
    yInRange = all(verts(:,2) >= osmYRange(1)) && all(verts(:,2) <= osmYRange(2));
    zHeightEnough = max(verts(:,3)) > minBuildingHeight;
    
    if xInRange && yInRange && zHeightEnough
        buildingMeshes = [buildingMeshes, currentMesh]; %#ok<AGROW>
    end
end

if isempty(buildingMeshes)
    error('No building meshes filtered!');
else
    fprintf('  ✓ Filtered %d building meshes\n', length(buildingMeshes));
end

%% 6. Calculate occupancyMap3D Range
fprintf('6. Calculating occupancyMap3D range...\n');

allVertices = [];
for i = 1:length(buildingMeshes)
    verts = buildingMeshes(i).Vertices;
    allVertices = [allVertices; verts]; %#ok<AGROW>
end

xMin = min(allVertices(:,1)) - 10;
xMax = max(allVertices(:,1)) + 10;
yMin = min(allVertices(:,2)) - 10;
yMax = max(allVertices(:,2)) + 10;
zMin = 0;
zMax = max(allVertices(:,3)) + 20;

fprintf('  Map Range: X[%.1f, %.1f], Y[%.1f, %.1f], Z[%.1f, %.1f]\n', ...
    xMin, xMax, yMin, yMax, zMin, zMax);

%% 7. Calculate Grid Center Points
fprintf('7. Calculating grid center points...\n');
resolution = 2.0;

xEdges = xMin:resolution:xMax;
yEdges = yMin:resolution:yMax;
zEdges = zMin:resolution:zMax;

xCenters = (xEdges(1:end-1) + xEdges(2:end)) / 2;
yCenters = (yEdges(1:end-1) + yEdges(2:end)) / 2;
zCenters = (zEdges(1:end-1) + zEdges(2:end)) / 2;

xSize = length(xEdges) - 1;
ySize = length(yEdges) - 1;
zSize = length(zEdges) - 1;

fprintf('  Grid Size: [%d, %d, %d], Resolution: %.1f m\n', xSize, ySize, zSize, resolution);

%% 8. Create occupancyMap3D
fprintf('8. Creating occupancyMap3D...\n');

try
    omap = occupancyMap3D(1/resolution);
    omap.FreeThreshold = 0.5;
    fprintf('  Creation Method: occupancyMap3D(1/resolution)\n');
catch
    try
        omap = occupancyMap3D('Resolution', resolution);
        omap.FreeThreshold = 0.5;
        fprintf('  Creation Method: occupancyMap3D(''Resolution'', resolution)\n');
    catch
        omap = occupancyMap3D();
        if isprop(omap, 'Resolution')
            omap.Resolution = resolution;
        end
        if isprop(omap, 'FreeThreshold')
            omap.FreeThreshold = 0.5;
        end
        fprintf('  Creation Method: occupancyMap3D() + manual settings\n');
    end
end

if isprop(omap, 'GridOriginInWorld')
    omap.GridOriginInWorld = [xMin, yMin, zMin];
elseif isprop(omap, 'LocalOriginInWorld')
    omap.LocalOriginInWorld = [xMin, yMin, zMin];
end

if isprop(omap, 'GridSize')
    omap.GridSize = [xSize, ySize, zSize];
end

fprintf('  ✓ occupancyMap3D initialization complete\n');

%% 9. Fill Building Voxels
fprintf('9. Filling building voxels...\n');

h = waitbar(0, 'Processing building outlines...');

for i = 1:length(buildingMeshes)
    mesh = buildingMeshes(i);
    vertices = mesh.Vertices;
    
    if isempty(vertices)
        continue;
    end

    meshZMin = min(vertices(:,3));
    meshZMax = max(vertices(:,3));
    xyPoints = vertices(:,1:2);
    
    if size(xyPoints, 1) > 2
        try
            k = convhull(xyPoints(:,1), xyPoints(:,2));
            buildingOutline = xyPoints(k, :);
        catch
            buildingOutline = [
                min(xyPoints(:,1)), min(xyPoints(:,2));
                max(xyPoints(:,1)), min(xyPoints(:,2));
                max(xyPoints(:,1)), max(xyPoints(:,2));
                min(xyPoints(:,1)), max(xyPoints(:,2));
                min(xyPoints(:,1)), min(xyPoints(:,2))
            ];
        end
    else
        buildingOutline = [
            min(xyPoints(:,1)), min(xyPoints(:,2));
            max(xyPoints(:,1)), min(xyPoints(:,2));
            max(xyPoints(:,1)), max(xyPoints(:,2));
            min(xyPoints(:,1)), max(xyPoints(:,2));
            min(xyPoints(:,1)), min(xyPoints(:,2))
        ];
    end
    
    [X, Y] = meshgrid(xCenters, yCenters);
    inBuilding = inpolygon(X, Y, buildingOutline(:,1), buildingOutline(:,2));
    
    [rows, cols] = find(inBuilding);
    for j = 1:length(rows)
        xWorld = xCenters(cols(j));
        yWorld = yCenters(rows(j));
        
        zValues = zCenters(zCenters >= meshZMin & zCenters <= meshZMax);
        for k2 = 1:length(zValues)
            setOccupancy(omap, [xWorld, yWorld, zValues(k2)], 1);
        end
    end
    
    waitbar(i/length(buildingMeshes), h, sprintf('Processing building %d/%d', i, length(buildingMeshes)));
end

close(h);
inflate(omap, 2.0);

fprintf('  ✓ occupancyMap3D complete (%d buildings, inflated 2m)\n\n', length(buildingMeshes));

%% ============================================================
%% 10. Task Allocation and Global Path Planning (Using CONFIG)
%% ============================================================

fprintf('\n====================================================\n');
fprintf('10. Task Allocation and Global Path Planning\n');
fprintf('====================================================\n\n');

globalRefPaths = [];
assignment     = [];
charger_pos    = [];
taskInfo       = [];

%% ----------------------------------------------------------------
%% 10.1 Generate Charging Stations (Auto-configured based on CONFIG)
%% ----------------------------------------------------------------
fprintf('[Step 1] Generating charging stations...\n');

numUAVs     = CONFIG.numUAVs;      % Number of UAVs
numChargers = CONFIG.numChargers;  % Number of charging stations

% Automatically select layout mode based on number of charging stations
if strcmp(CONFIG.charger_mode, 'auto')
    if numChargers <= 4
        actual_mode = 'corners';
    elseif numChargers == 9
        actual_mode = 'grid_3x3';
    elseif numChargers == 16
        actual_mode = 'grid_4x4';
    else
        actual_mode = 'grid';
    end
else
    actual_mode = CONFIG.charger_mode;
end

fprintf('  Layout Mode: %s (Charging Stations: %d, UAVs: %d)\n', actual_mode, numChargers, numUAVs);
    
% Generate charging station positions (evenly distributed within city scene range)
switch actual_mode
    case 'corners'
        temp = [
            xMin + 50,  yMin + 50,  CONFIG.charger_height;
            xMax - 50,  yMin + 50,  CONFIG.charger_height;
            xMin + 50,  yMax - 50,  CONFIG.charger_height;
            xMax - 50,  yMax - 50,  CONFIG.charger_height
        ];
        charger_pos = temp(1:min(numChargers, 4), :);

    case 'grid_3x3'
        grid_x = linspace(xMin + 100, xMax - 100, 3);
        grid_y = linspace(yMin + 100, yMax - 100, 3);
        charger_pos = [];
        for i = 1:3
            for j = 1:3
                if size(charger_pos, 1) < numChargers
                    charger_pos = [charger_pos; ...
                        grid_x(i), grid_y(j), CONFIG.charger_height]; %#ok<AGROW>
                end
            end
        end

    case 'grid_4x4'
        grid_x = linspace(xMin + 80, xMax - 80, 4);
        grid_y = linspace(yMin + 80, yMax - 80, 4);
        charger_pos = [];
        for i = 1:4
            for j = 1:4
                if size(charger_pos, 1) < numChargers
                    charger_pos = [charger_pos; ...
                        grid_x(i), grid_y(j), CONFIG.charger_height]; %#ok<AGROW>
                end
            end
        end

    case 'grid'
        grid_size = ceil(sqrt(numChargers));
        grid_x = linspace(xMin + 100, xMax - 100, grid_size);
        grid_y = linspace(yMin + 100, yMax - 100, grid_size);
        charger_pos = [];
        for i = 1:grid_size
            for j = 1:grid_size
                if size(charger_pos, 1) < numChargers
                    charger_pos = [charger_pos; ...
                        grid_x(i), grid_y(j), CONFIG.charger_height]; %#ok<AGROW>
                end
            end
        end

    case 'circle'
        center_x = (xMin + xMax) / 2;
        center_y = (yMin + yMax) / 2;
        radius   = min(xMax - xMin, yMax - yMin) / 2 - 100;
        charger_pos = zeros(numChargers, 3);
        for i = 1:numChargers
            angle = 2 * pi * (i-1) / numChargers;
            charger_pos(i, :) = [
                center_x + radius * cos(angle), ...
                center_y + radius * sin(angle), ...
                CONFIG.charger_height
            ];
        end

    otherwise
        error('Unknown charging station layout mode: %s', actual_mode);
end
    
fprintf('  Actual charging stations generated: %d, Number of UAVs: %d\n', ...
    size(charger_pos, 1), numUAVs);
    
% Validate charging station positions
fprintf('  Validating charging station positions...\n');
valid_chargers = [];
for cc = 1:size(charger_pos, 1)
    try
        occ_val = checkOccupancy(omap, charger_pos(cc, :));
        if occ_val < 0.5
            valid_chargers = [valid_chargers; charger_pos(cc, :)]; %#ok<AGROW>
        else
            % Inside building, adjust height
            adjusted = charger_pos(cc, :);
            adjusted(3) = zMax - 20;
            valid_chargers = [valid_chargers; adjusted]; %#ok<AGROW>
            fprintf('    Charger %d: Height adjusted to %.1fm\n', cc, adjusted(3));
        end
    catch
        valid_chargers = [valid_chargers; charger_pos(cc, :)]; %#ok<AGROW>
    end
end
charger_pos = valid_chargers;

fprintf('  ✓ Charging station setup complete: %d stations\n', size(charger_pos, 1));
fprintf('  Layout: %s, Height: %.1fm\n\n', actual_mode, CONFIG.charger_height);
    
    
%% ----------------------------------------------------------------
%% 10.2 Generate Task Points (Based on CONFIG)
%% ----------------------------------------------------------------
fprintf('[Step 2] Generating task points...\n');
    
numTasks = CONFIG.numTasks;  % Use configuration from CONFIG
    
switch CONFIG.task_mode
    case 'random'
        % Random generation (avoiding buildings)
        tasks_3d   = zeros(numTasks, 3);
        generated  = 0;
        maxAttempts = numTasks * 100;
        attempts    = 0;
        
        fprintf('  Generation Range:\n');
        fprintf('    X: [%.1f, %.1f]m\n', xMin, xMax);
        fprintf('    Y: [%.1f, %.1f]m\n', yMin, yMax);
        fprintf('    Z: [%.1f, %.1f]m\n', CONFIG.task_z_min, CONFIG.task_z_max);
        fprintf('  Starting generation...\n');
        
        while generated < numTasks && attempts < maxAttempts
            attempts = attempts + 1;
            
            x = xMin + rand() * (xMax - xMin);
            y = yMin + rand() * (yMax - yMin);
            z = CONFIG.task_z_min + rand() * (CONFIG.task_z_max - CONFIG.task_z_min);
            
            candidate = [x, y, z];
            
            try
                occStatus = checkOccupancy(omap, candidate);
                if occStatus < 0.5
                    generated = generated + 1;
                    tasks_3d(generated, :) = candidate;
                    
                    if mod(generated, 5) == 0
                        fprintf('    Progress: %d/%d\n', generated, numTasks);
                    end
                end
            catch
                continue;
            end
        end
        
        if generated < numTasks
            warning('Only generated %d/%d task points', generated, numTasks);
            tasks_3d = tasks_3d(1:generated, :);
            numTasks = generated;
        end
        
    case 'custom'
        error('Custom mode requires defining tasks_3d in code');
        
    otherwise
        error('Unknown task point generation mode: %s', CONFIG.task_mode);
end
    
fprintf('  ✓ Task point generation complete: %d points\n', numTasks);
fprintf('  Actual Height: [%.1f, %.1f]m\n\n', ...
    min(tasks_3d(:,3)), max(tasks_3d(:,3)));
    
    
%% ----------------------------------------------------------------
%% 10.3 MTSP Optimization (Using CONFIG Parameters)
%% ----------------------------------------------------------------
fprintf('[Step 3] MTSP Route Optimization...\n\n');
    
taskDef = struct();
taskDef.positions    = tasks_3d;
taskDef.durations    = 60 * ones(numTasks, 1);
taskDef.time_offsets = zeros(numTasks, 1);

opts_mtsp = struct();
opts_mtsp.mapBounds       = [xMin, xMax; yMin, yMax; zMin, zMax];
opts_mtsp.omap            = omap;
opts_mtsp.velocity        = CONFIG.speed_limit;
opts_mtsp.t_c             = 300;
opts_mtsp.max_endurance   = 1800;
opts_mtsp.PopulationSize  = CONFIG.ga_population;
opts_mtsp.Generations     = CONFIG.ga_generations;
opts_mtsp.q1              = CONFIG.mtsp_q1;
opts_mtsp.q2              = CONFIG.mtsp_q2;
opts_mtsp.q3              = CONFIG.mtsp_q3;
opts_mtsp.distanceMetric  = CONFIG.distance_metric;
opts_mtsp.verticalPenalty = CONFIG.vertical_penalty;
    
try
    [assignment, bestFitness, simResults] = MTSP(...
        numUAVs, taskDef, charger_pos, [], opts_mtsp);
    
    fprintf('✓ MTSP optimization successful\n');
    fprintf('  Best Fitness: %.6f\n', bestFitness);
    fprintf('  Total Flight Distance: %.1f m (%.2f km)\n', ...
        simResults.total_distance, simResults.total_distance/1000);
    fprintf('  Estimated Completion Time: %.1f minutes\n', simResults.max_completion_time/60);
    fprintf('  UAVs Used: %d/%d\n\n', ...
        length(simResults.used_chargers), numUAVs);
    
    save('mtsp_results.mat', 'assignment', 'bestFitness', ...
        'simResults', 'taskDef', 'charger_pos', 'opts_mtsp');
    
    taskInfo = struct();
    taskInfo.tasks = struct();
    taskInfo.tasks.positions    = tasks_3d;
    taskInfo.tasks.time_offsets = taskDef.time_offsets;
    taskInfo.tasks.durations    = taskDef.durations;
    taskInfo.assignment         = assignment;

    % Generate execution sequence points from MTSP results (including task points + mid-route returns + final return)
    if exist('simResults','var') && isfield(simResults,'results')
        mtspPtsSeq = buildPtsSeqFromMTSP(simResults, charger_pos);
        fprintf('  ✓ Generated mtspPtsSeq from MTSP results (including task points + mid-route returns + final return)\n');
    else
        mtspPtsSeq = cell(numUAVs,1);
        fprintf('  ⚠ simResults does not exist, mtspPtsSeq is empty\n');
    end

catch ME
    fprintf('✗ MTSP optimization failed: %s\n', ME.message);
    fprintf('  Using random assignment\n\n');
    
    assignment = randi([1, numUAVs], numTasks, 1);
    
    taskInfo = struct();
    taskInfo.tasks = struct();
    taskInfo.tasks.positions    = tasks_3d;
    taskInfo.tasks.time_offsets = zeros(numTasks, 1);
    taskInfo.tasks.durations    = 60 * ones(numTasks, 1);
    taskInfo.assignment         = assignment;

    mtspPtsSeq = cell(numUAVs,1);
end
    
    
%% ----------------------------------------------------------------
%% 10.4-10.6 BiRRT Path Planning
%% ----------------------------------------------------------------
fprintf('[Step 4] BiRRT Global Path Planning...\n');

globalRefPaths   = cell(numUAVs, 1);
segmentRefPaths  = cell(numUAVs, 1);  % Each UAV has a cell, storing BiRRT path for each segment

if ~isempty(assignment) && ~isempty(taskInfo)
    events_pos   = taskInfo.tasks.positions;
    time_offsets = taskInfo.tasks.time_offsets;
    
    for u = 1:numUAVs

        % Prefer using execution sequence points from MTSP
        if exist('mtspPtsSeq','var') && ~isempty(mtspPtsSeq{u})
            ptsSeq = mtspPtsSeq{u};
            fprintf('  UAV%d: Using MTSP sequence points (with returns), Total Points = %d, Path Segments = %d\n', ...
                u, size(ptsSeq,1), max(size(ptsSeq,1)-1, 0));
        else
            % Fall back to original logic if MTSP results unavailable: only use task points
            assigned = find(assignment == u);

            if isempty(assigned)
                globalRefPaths{u} = [];
                fprintf('  UAV%d: No tasks assigned\n', u);
                continue;
            end

            [~, sidx] = sort(time_offsets(assigned));
            seq   = assigned(sidx);
            ptsSeq = [charger_pos(u, :); events_pos(seq, :)];

            fprintf('  UAV%d: Not using MTSP sequence points, fallback to charger->tasks (%d segments)\n', ...
                u, size(ptsSeq, 1)-1);
        end

        pathAll = [];
        segmentRefPaths{u} = {};   % Initialize segment path cell for current UAV

        for seg = 1:(size(ptsSeq, 1) - 1)
            sp = ptsSeq(seg, :);
            gp = ptsSeq(seg + 1, :);

            fprintf('    Planning segment %d: [%.1f,%.1f,%.1f] -> [%.1f,%.1f,%.1f]...\n', ...
                seg, sp, gp);

            try
                sp_occ = checkOccupancy(omap, sp);
                gp_occ = checkOccupancy(omap, gp);
                if sp_occ >= 0.5, sp(3) = sp(3) + 50; end
                if gp_occ >= 0.5, gp(3) = gp(3) + 50; end
            catch
            end

            try
                height_diff = abs(gp(3) - sp(3));

                if height_diff > 150
                    fprintf('      -> Height diff %.1fm, splitting into two segments\n', height_diff);
                    mid_z  = (sp(3) + gp(3)) / 2;
                    mid_pt = [gp(1), gp(2), mid_z];
                    seg1 = birrt_plan_segment(sp, mid_pt, ...
                        xMin, xMax, yMin, yMax, zMin, zMax, omap);
                    if ~isempty(seg1)
                        seg2 = birrt_plan_segment(mid_pt, gp, ...
                            xMin, xMax, yMin, yMax, zMin, zMax, omap);
                        if ~isempty(seg2)
                            segPath = [seg1; seg2(2:end, :)];
                            fprintf('      ✓ Success (%d points)\n', size(segPath, 1));
                        else
                            segPath = [];
                        end
                    else
                        segPath = [];
                    end
                else
                    segPath = birrt_plan_segment(sp, gp, ...
                        xMin, xMax, yMin, yMax, zMin, zMax, omap);
                    if ~isempty(segPath)
                        fprintf('      ✓ Success (%d points)\n', size(segPath, 1));
                    end
                end

                if isempty(segPath)
                    fprintf('      ⚠ BiRRT failed, using linear interpolation\n');
                    dist   = norm(gp - sp);
                    numPts = max(5, ceil(dist / 10));
                    segPath = sp + (gp - sp) .* linspace(0, 1, numPts)';
                end

                % Save this segment's BiRRT path (whether planning succeeded or fallback to line)
                segmentRefPaths{u}{seg} = segPath;

                if isempty(pathAll)
                    pathAll = segPath;
                else
                    pathAll = [pathAll; segPath(2:end, :)]; %#ok<AGROW>
                end

            catch ME
                fprintf('      ⚠ Exception: %s\n', ME.message);
                dist   = norm(gp - sp);
                numPts = max(5, ceil(dist / 10));
                segPath = sp + (gp - sp) .* linspace(0, 1, numPts)';

                segmentRefPaths{u}{seg} = segPath;

                if isempty(pathAll)
                    pathAll = segPath;
                else
                    pathAll = [pathAll; segPath(2:end, :)]; %#ok<AGROW>
                end
            end
        end

        globalRefPaths{u} = pathAll;
        if ~isempty(pathAll) && size(pathAll, 1) >= 2
            fprintf('  UAV%d: ✓ Path complete (%d points)\n', u, size(pathAll, 1));
        else
            fprintf('  UAV%d: ⚠ Path failed\n', u);
        end
    end
    
    nSuccessful = sum(cellfun(@(c) ~isempty(c) && size(c,1)>=2, globalRefPaths));
    fprintf('\n  ✓ Path planning complete: %d/%d UAVs\n', nSuccessful, numUAVs);
else
    fprintf('  ✗ No valid task assignment\n');
end


%% ============================================================
%% 10.7 Minimum Snap Global Smoothing by Task Segment
%% ============================================================
fprintf('\n[Step 5] Performing Minimum Snap global smoothing by task segment...\n');

if ~CONFIG.minimum_snap.enabled || isempty(taskInfo) || ~isfield(taskInfo, 'assignment')
    fprintf('  Minimum Snap not enabled or task info missing, optimizedRefPaths=globalRefPaths\n');
    optimizedRefPaths = globalRefPaths;
else
    optimizedRefPaths = buildSegmentedMinSnapPaths( ...
        globalRefPaths, segmentRefPaths, mtspPtsSeq, CONFIG, omap);
end

fprintf('  Segment Minimum Snap complete, using optimizedRefPaths as global reference\n');

% Clean consecutive duplicate points in optimized paths before using as reference
for u = 1:CONFIG.numUAVs
    if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u},1) > 1
        optimizedRefPaths{u} = removeConsecutiveDuplicates(optimizedRefPaths{u}, 1e-6);
    end
end


%% ============================================================
%% 10.8 Preview Minimum Snap Reference Paths
%% ============================================================
fprintf('\n10.8 Previewing Minimum Snap reference paths...\n');

previewFig = figure('Name', 'Minimum Snap Global Reference Path Preview', ...
    'Position', [80, 80, 1200, 800], ...
    'Color', 'w');
previewAx = axes('Parent', previewFig);
hold(previewAx, 'on');
grid(previewAx, 'on');
axis(previewAx, 'equal');
xlabel(previewAx, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(previewAx, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel(previewAx, 'Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
title(previewAx, 'Minimum Snap Global Reference Path (Pre-simulation Preview)', ...
    'FontSize', 14, 'FontWeight', 'bold');

trajColors = {
    [1 0 1],      % Magenta
    [0 1 1],      % Cyan
    [1 1 0],      % Yellow
    [1 0.5 0],    % Orange
    [0.5 0 1],    % Purple
    [0 1 0.5],    % Teal
    [1 0.5 0.5],  % Pink
    [0.5 1 0],    % Lime
    [0 0.5 1]     % Blue
};

refCount_preview = 0;

for u = 1:numUAVs
    if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1
        refPath   = optimizedRefPaths{u};
        refLabel  = sprintf('UAV%d MinSnap Ref', u);
        lineStyle = '-.';    % Dash-dot line
        lineWidth = 2.5;
    elseif ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1
        refPath   = globalRefPaths{u};
        refLabel  = sprintf('UAV%d BiRRT Ref', u);
        lineStyle = '--';    % Dashed line
        lineWidth = 2.0;
    else
        continue;
    end
    
    colorIdx = mod(u-1, length(trajColors)) + 1;
    
    plot3(previewAx, ...
        refPath(:,1), refPath(:,2), refPath(:,3), ...
        lineStyle, ...
        'LineWidth', lineWidth, ...
        'Color', trajColors{colorIdx}, ...
        'DisplayName', refLabel);
    
    % Start point marker (charging station)
    if ~isempty(charger_pos) && u <= size(charger_pos,1)
        scatter3(previewAx, ...
            charger_pos(u,1), charger_pos(u,2), charger_pos(u,3), ...
            80, 'g', 'filled', ...
            'MarkerEdgeColor', 'k', ...
            'LineWidth', 1.5);
    end
    
    % End point marker (reference path end)
    scatter3(previewAx, ...
        refPath(end,1), refPath(end,2), refPath(end,3), ...
        80, 'r', 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.5);
    
    refCount_preview = refCount_preview + 1;
end

if ~isempty(taskInfo) && isfield(taskInfo, 'tasks') && ...
   isfield(taskInfo.tasks, 'positions')
    tasks_pos = taskInfo.tasks.positions;
    scatter3(previewAx, ...
        tasks_pos(:,1), tasks_pos(:,2), tasks_pos(:,3), ...
        50, '^', 'r', 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0, ...
        'DisplayName', 'Task Points');
end

xlim(previewAx, [xMin, xMax]);
ylim(previewAx, [yMin, yMax]);
zlim(previewAx, [zMin, zMax]);
view(previewAx, 45, 30);

lgd_prev = legend(previewAx, 'Location', 'northeastoutside');
lgd_prev.FontSize = 9;
lgd_prev.Box = 'on';

fprintf('  ✓ Preview complete: %d reference paths plotted\n', refCount_preview);

%% ============================================================
%% X. Preview BiRRT Original Paths (Without Minimum Snap)
%% ============================================================
fprintf('\nX. Previewing BiRRT original paths (without Minimum Snap smoothing)...\n');

if isempty(globalRefPaths)
    fprintf('  ⚠ globalRefPaths is empty, no BiRRT paths found\n');
else
    fig_birrt = figure('Name', 'BiRRT Original Global Path Preview', ...
        'Position', [100, 100, 1200, 800], ...
        'Color', 'w');
    ax_birrt = axes('Parent', fig_birrt);
    hold(ax_birrt, 'on');
    grid(ax_birrt, 'on');
    axis(ax_birrt, 'equal');
    xlabel(ax_birrt, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax_birrt, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax_birrt, 'Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title(ax_birrt, 'BiRRT Original Path Preview (Global Polyline Only)', ...
        'FontSize', 14, 'FontWeight', 'bold');

    trajColors = {
        [1 0 1],      % Magenta
        [0 1 1],      % Cyan
        [1 1 0],      % Yellow
        [1 0.5 0],    % Orange
        [0.5 0 1],    % Purple
        [0 1 0.5],    % Teal
        [1 0.5 0.5],  % Pink
        [0.5 1 0],    % Lime
        [0 0.5 1]     % Blue
    };

    numUAVs     = numel(globalRefPaths);
    birrtCount  = 0;

    for u = 1:numUAVs
        if ~isempty(globalRefPaths{u}) && size(globalRefPaths{u},1) > 1
            path_u   = globalRefPaths{u};
            colorIdx = mod(u-1, length(trajColors)) + 1;

            plot3(ax_birrt, ...
                path_u(:,1), path_u(:,2), path_u(:,3), ...
                '--', ...
                'LineWidth', 2.0, ...
                'Color', trajColors{colorIdx}, ...
                'DisplayName', sprintf('UAV%d BiRRT', u));

            if ~isempty(charger_pos) && u <= size(charger_pos,1)
                scatter3(ax_birrt, ...
                    charger_pos(u,1), charger_pos(u,2), charger_pos(u,3), ...
                    70, 'g', 'filled', ...
                    'MarkerEdgeColor', 'k', ...
                    'LineWidth', 1.5);
            end

            scatter3(ax_birrt, ...
                path_u(end,1), path_u(end,2), path_u(end,3), ...
                70, 'r', 'filled', ...
                'MarkerEdgeColor', 'k', ...
                'LineWidth', 1.5);

            birrtCount = birrtCount + 1;
        end
    end

    if ~isempty(taskInfo) && isfield(taskInfo, 'tasks') && ...
       isfield(taskInfo.tasks, 'positions')
        tasks_pos = taskInfo.tasks.positions;
        scatter3(ax_birrt, ...
            tasks_pos(:,1), tasks_pos(:,2), tasks_pos(:,3), ...
            50, '^', 'r', 'filled', ...
            'MarkerEdgeColor', 'k', ...
            'LineWidth', 1.0, ...
            'DisplayName', 'Task Points');
    end

    if exist('xMin','var') && exist('xMax','var')
        xlim(ax_birrt, [xMin, xMax]);
    end
    if exist('yMin','var') && exist('yMax','var')
        ylim(ax_birrt, [yMin, yMax]);
    end
    if exist('zMin','var') && exist('zMax','var')
        zlim(ax_birrt, [zMin, zMax]);
    end
    view(ax_birrt, 45, 30);

    lgd_b = legend(ax_birrt, 'Location', 'northeastoutside');
    lgd_b.FontSize = 9;
    lgd_b.Box = 'on';

    fprintf('  ✓ BiRRT original path preview complete, %d paths plotted\n', birrtCount);
end
%% ============================================================
%% X.1 MTSP Trajectory Visualization (Task vs Charger)
%% ============================================================
fprintf('\nX.1 Visualizing MTSP trajectories (Task vs Charger segments)...\n');

if exist('simResults', 'var') && isfield(simResults, 'results') && ~isempty(simResults.results)
    
    fig_mtsp = figure('Name', 'MTSP Trajectory Visualization', ...
        'Position', [150, 100, 1200, 800], ...
        'Color', 'w');
    ax_mtsp = axes('Parent', fig_mtsp);
    hold(ax_mtsp, 'on');
    grid(ax_mtsp, 'on');
    axis(ax_mtsp, 'equal');
    xlabel(ax_mtsp, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax_mtsp, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax_mtsp, 'Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title(ax_mtsp, 'MTSP Trajectory: To Task (Red) vs To Charger (Blue)', ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    % Trajectory colors
    taskColor = [0.8, 0.0, 0.0];     % Red - To Task
    chargerColor = [0.0, 0.0, 0.8];  % Blue - To Charger
    
    numUAVs_mtsp = numel(simResults.results);
    taskSegCount = 0;
    chargerSegCount = 0;
    
    legendHandles = [];
    legendLabels = {};
    hasTaskLegend = false;
    hasChargerLegend = false;
    
    for u = 1:numUAVs_mtsp
        uLog = simResults.results{u};
        if isempty(uLog), continue; end
        
        % Start from charger
        if u <= size(charger_pos, 1)
            currPos = charger_pos(u, :);
        else
            currPos = [0, 0, 0];
        end
        
        for k = 1:numel(uLog)
            e = uLog{k};
            
            switch e.type
                case 'to_task'
                    % To Task - Red solid line
                    targetPos = e.position;
                    h = plot3(ax_mtsp, ...
                        [currPos(1), targetPos(1)], ...
                        [currPos(2), targetPos(2)], ...
                        [currPos(3), targetPos(3)], ...
                        '-', 'LineWidth', 2.0, 'Color', taskColor);
                    currPos = targetPos;
                    taskSegCount = taskSegCount + 1;
                    
                    if ~hasTaskLegend
                        legendHandles(end+1) = h;
                        legendLabels{end+1} = 'To Task';
                        hasTaskLegend = true;
                    end
                    
                case {'to_charger', 'return_final'}
                    % To Charger - Blue solid line
                    fromPos = e.from;
                    toPos = e.to;
                    h = plot3(ax_mtsp, ...
                        [fromPos(1), toPos(1)], ...
                        [fromPos(2), toPos(2)], ...
                        [fromPos(3), toPos(3)], ...
                        '-', 'LineWidth', 2.0, 'Color', chargerColor);
                    currPos = toPos;
                    chargerSegCount = chargerSegCount + 1;
                    
                    if ~hasChargerLegend
                        legendHandles(end+1) = h;
                        legendLabels{end+1} = 'To Charger';
                        hasChargerLegend = true;
                    end
                    
                case 'task'
                    currPos = e.position;
                    
                case 'charging'
                    currPos = e.at;
            end
        end
    end
    
    % Plot task points
    if exist('taskInfo', 'var') && isfield(taskInfo, 'tasks') && isfield(taskInfo.tasks, 'positions')
        tasks_pos = taskInfo.tasks.positions;
        h_tasks = scatter3(ax_mtsp, ...
            tasks_pos(:,1), tasks_pos(:,2), tasks_pos(:,3), ...
            120, 'p', 'MarkerFaceColor', [1, 0.8, 0], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        legendHandles(end+1) = h_tasks;
        legendLabels{end+1} = 'Task Points';
        
        for t = 1:size(tasks_pos, 1)
            text(ax_mtsp, tasks_pos(t,1)+5, tasks_pos(t,2)+5, tasks_pos(t,3), ...
                sprintf('T%d', t), 'FontSize', 8, 'FontWeight', 'bold');
        end
    end
    
    % Plot charger positions
    if exist('charger_pos', 'var') && ~isempty(charger_pos)
        h_chargers = scatter3(ax_mtsp, ...
            charger_pos(:,1), charger_pos(:,2), charger_pos(:,3), ...
            150, 's', 'MarkerFaceColor', [0.2, 0.8, 0.2], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
        legendHandles(end+1) = h_chargers;
        legendLabels{end+1} = 'Charging Stations';
        
        for c = 1:size(charger_pos, 1)
            text(ax_mtsp, charger_pos(c,1)+5, charger_pos(c,2)-5, charger_pos(c,3), ...
                sprintf('C%d', c), 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0, 0.5, 0]);
        end
    end
    
    % Set axis limits
    if exist('xMin', 'var') && exist('xMax', 'var')
        xlim(ax_mtsp, [xMin, xMax]);
    end
    if exist('yMin', 'var') && exist('yMax', 'var')
        ylim(ax_mtsp, [yMin, yMax]);
    end
    if exist('zMin', 'var') && exist('zMax', 'var')
        zlim(ax_mtsp, [zMin, zMax]);
    end
    
    view(ax_mtsp, 45, 30);
    
    if ~isempty(legendHandles)
        lgd_mtsp = legend(ax_mtsp, legendHandles, legendLabels, 'Location', 'northeastoutside');
        lgd_mtsp.FontSize = 9;
        lgd_mtsp.Box = 'on';
    end
    
    fprintf('  ✓ MTSP trajectory visualization complete\n');
    fprintf('    To Task segments: %d (Red)\n', taskSegCount);
    fprintf('    To Charger segments: %d (Blue)\n', chargerSegCount);
    
else
    fprintf('  ⚠ simResults not available, skipping MTSP trajectory visualization\n');
end
%% ============================================================
%% 11. Setup Multi-UAV Dynamic Obstacle Avoidance System (Corrected Version)
%% ============================================================
fprintf('\n11. Setting up multi-UAV dynamic obstacle avoidance system...\n'); 
numUAVs              = CONFIG.numUAVs; 
egoUAVs              = cell(numUAVs, 1); 
sensors              = cell(numUAVs, 1); 
trackers             = cell(numUAVs, 1); 
planners             = cell(numUAVs, 1); 
collisionValidators  = cell(numUAVs, 1); 

for u = 1:numUAVs
    % Create UAV platform
    egoUAVs{u} = uavPlatform(sprintf("UAV_%d", u), scene, "ReferenceFrame", "ENU"); 
    updateMesh(egoUAVs{u}, "quadrotor", {5}, [0 0 1], eye(4)); 
    
    fprintf('  UAV%d: Setting up sensors...\n', u); 
    
    % Setup sensors
    [sensors{u}.lidar, sensors{u}.radar, ... 
     sensors{u}.radarLoc, sensors{u}.radarAng] = helperSetupSensors(egoUAVs{u}); 
    
    % Create trackers
    [sensors{u}.lidarDetector, trackers{u}.lidarJPDA, ... 
     trackers{u}.radarPHD, trackers{u}.grid] = ... 
        helperCreateTrackers(sensors{u}.lidar, egoUAVs{u}, ... 
        sensors{u}.radarLoc, sensors{u}.radarAng); 
    
    % *** Create MPC planner (complete version) ***
    if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1
        refPath = optimizedRefPaths{u};
        pathType = 'Optimized MinSnap';
    elseif ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1
        refPath = globalRefPaths{u};
        pathType = 'BiRRT';
    else
        refPath = [];
        pathType = 'None';
    end
    
    if ~isempty(refPath) && size(refPath, 1) > 1
        % Convert to standard refPath structure
        refPath_struct = convertToRefPathStruct(refPath);
        
        % Create complete MPC planner structure
        planners{u} = struct();
        planners{u}.refPath = refPath_struct;
        planners{u}.timeHorizon = 3.0;           % Planning horizon 3 seconds
        planners{u}.dt = 0.2;                     % Time step 0.1 seconds
        planners{u}.numSpeeds = 3;                % Speed sampling count
        planners{u}.numLateralOffsets = 3;        % Lateral offset sampling count
        planners{u}.lateralRange = [-2, 2];       % Lateral offset range ±2 meters
        
        fprintf('    ✓ Using %s path (%d points, %.1fm)\n', ...
            pathType, size(refPath, 1), refPath_struct.sMax);
    else
        planners{u} = [];
        fprintf('    ⚠ No global path\n');
    end
    
    % Create collision validator
    vehDims = struct('Length', 5, 'Width', 5, 'Height', 3); 
    collisionValidators{u} = helperCreateCollisionValidator(trackers{u}.grid, vehDims); 
    collisionValidators{u}.tracker = trackers{u}.lidarJPDA; 
    
    fprintf('  ✓ UAV%d initialization complete\n', u); 
end

fprintf('  ✓ All UAVs initialized\n'); 

% *** Remove "pre-conversion" code at beginning of Section 12 (already converted in Section 11) ***
fprintf('  ✓ All planner refPaths pre-converted\n\n');
%% ============================================================
%% 12. Main Simulation Loop (Optimized Memory and Performance Version - Optimized Output and Sensor Updates)
%% ============================================================
setup(scene); 

% ===== Performance Optimization Configuration =====
OUTPUT_INTERVAL = 500;      % Output interval (changed from 100 to 500)
SENSOR_UPDATE_INTERVAL = 3; % Sensor update interval (update every 3 steps)

% ===== 1. Initialize Segmented Trajectory System =====
segmentTrajectories = cell(numUAVs, 1); 
currentSegmentIdx = ones(numUAVs, 1); 
segmentCompleted = false(numUAVs, 1); 

% ===== 2. Initialize Segment State Management =====
segmentStates = cell(numUAVs, 1); 
for u = 1:numUAVs
    segmentStates{u} = struct(); 
    segmentStates{u}.nextSegmentAvailable = false; 
    segmentStates{u}.nextSegmentIndex = []; 
    segmentStates{u}.nextSegmentTrajectory = []; 
    segmentStates{u}.hasTransitioned = false; 
    segmentStates{u}.previousSegmentIndex = []; 
    segmentStates{u}.transitionPoint = []; 
end

% ===== 3. MPC Trajectory Management Variable Initialization =====
currentTrajectories = cell(numUAVs, 1); 
trajectoryStep = zeros(numUAVs, 1); 
stepsSinceReplan = zeros(numUAVs, 1); 
previousFullTrajectory = cell(numUAVs, 1); 
replanSteps = 5; 
uavArrived = false(numUAVs, 1); 
uavArrivalTime = zeros(numUAVs, 1); 

for u = 1:numUAVs
    currentTrajectories{u} = []; 
    previousFullTrajectory{u} = []; 
    trajectoryStep(u) = 0; 
    stepsSinceReplan(u) = 0; 
end

% ===== Sensor Cache Initialization =====
sensorCache = struct();
for u = 1:numUAVs
    sensorCache(u).radarTracks = objectTrack.empty;
    sensorCache(u).lidarTracks = objectTrack.empty;
    sensorCache(u).lastUpdateTime = -inf;
end

fprintf('  ✓ Segment state management initialized\n'); 
fprintf('  ✓ MPC trajectory management variables initialized\n'); 
fprintf('  ✓ Sensor cache initialized\n'); 

% ===== 4. Build Segmented Trajectories for Each UAV =====
for u = 1:numUAVs
    if ~isempty(optimizedRefPaths{u}) && ~isempty(mtspPtsSeq{u}) 
        segments = splitTrajectoryByMTSP(optimizedRefPaths{u}, mtspPtsSeq{u}); 
        segmentTrajectories{u} = segments; 
        fprintf('UAV%d: Trajectory split into %d segments\n', u, length(segments)); 
        
        initialPos = segments{1}(1, 1:3); 
        initialMotion = [initialPos, 0,0,0, 0,0,0, 1,0,0,0, 0,0,0]; 
        move(egoUAVs{u}, initialMotion); 
    else
        segmentTrajectories{u} = {}; 
        initialPos = charger_pos(u, :); 
        initialMotion = [initialPos, 0,0,0, 0,0,0, 1,0,0,0, 0,0,0]; 
        move(egoUAVs{u}, initialMotion); 
    end
end

% ===== 5. Simulation Parameters =====
speedLimit = CONFIG.speed_limit; 
accMax = CONFIG.acc_max; 
maxSteps = CONFIG.sim_time/2; 
logCount = 0; 

% ===== 6. System Log Initialization (Memory Optimized) =====
MEMORY_CONFIG.maxHistorySteps = min(500, CONFIG.sim_time);
systemLog = struct(); 
for u = 1:numUAVs
    systemLog.states{u} = zeros(MEMORY_CONFIG.maxHistorySteps, 6); 
    systemLog.bufferPointer{u} = 1; 
end

fprintf('\n12. Starting multi-UAV segmented trajectory simulation...\n'); 
fprintf('    Output Interval: %d steps | Sensor Update Interval: %d steps\n\n', OUTPUT_INTERVAL, SENSOR_UPDATE_INTERVAL);

%% ============================================================
%% Main Simulation Loop
%% ============================================================
isDecelerating = false(numUAVs, 1);  % Whether in deceleration phase
decelerationDir = zeros(numUAVs, 3);  % Deceleration direction (along trajectory)
while advance(scene) && logCount < maxSteps
    time = scene.CurrentTime; 
    logCount = logCount + 1; 
    
    % *** Optimization 2: Update sensors at intervals ***
    if mod(logCount, SENSOR_UPDATE_INTERVAL) == 0
        updateSensors(scene); 
    end
    
    for uavIdx = 1:numUAVs
        try
            % Skip completed UAVs
            if segmentCompleted(uavIdx) 
                continue; 
            end
            
            % ============================================================
            % 1. Read State (one-time read) 
            % ============================================================
            actualPose = read(egoUAVs{uavIdx}); 
            if size(actualPose, 1) > size(actualPose, 2) 
                actualPose = actualPose'; 
            end
            actualState = [actualPose(1:3), actualPose(4:6)]; 
            currentPos = actualState(1:3); 
            
            % ============================================================
            % 2. Segment State Management (extract local variables for optimization)
            % ============================================================
            currentSegIdx = currentSegmentIdx(uavIdx); 
            state = segmentStates{uavIdx};  % Extract to local variable
            
            if isempty(segmentTrajectories{uavIdx}) || ... 
               currentSegIdx > length(segmentTrajectories{uavIdx}) 
                segmentCompleted(uavIdx) = true; 
                continue; 
            end
            
            currentSegTraj = segmentTrajectories{uavIdx}{currentSegIdx}; 
            currentSegEnd = currentSegTraj(end, 1:3); 
            distToCurrentEnd = norm(currentPos - currentSegEnd); 
            
            % ============================================================
            % 3. Phase A->B: Preload Next Segment
            % ============================================================
            if distToCurrentEnd < 5.0 && ... 
               ~state.nextSegmentAvailable && ... 
               currentSegIdx < length(segmentTrajectories{uavIdx}) 
                
                nextSegIdx = currentSegIdx + 1; 
                nextSegTraj = segmentTrajectories{uavIdx}{nextSegIdx}; 
                
                state.nextSegmentAvailable = true; 
                state.nextSegmentIndex = nextSegIdx; 
                state.nextSegmentTrajectory = nextSegTraj; 
                state.transitionPoint = currentSegEnd; 
                segmentStates{uavIdx} = state;  % Write back
                
                % *** Optimization 1: Reduce output frequency ***
                if mod(logCount, OUTPUT_INTERVAL) == 0 || logCount <= 3
                    fprintf('[t=%.1fs] UAV%d State A->B: Preloading segment %d (%.1fm from current segment)\n', ... 
                        time, uavIdx, nextSegIdx, distToCurrentEnd); 
                end
            end
            
            % ============================================================
            % 4. Phase B->C: Reached Current Segment End, Switch Segment
            % ============================================================
            if state.nextSegmentAvailable && ~state.hasTransitioned
                currentSpeed = norm(actualState(4:6)); 
                
                if distToCurrentEnd < 0.8 && currentSpeed < 0.8
                    nextSegIdx = state.nextSegmentIndex; 
                    nextSegTraj = state.nextSegmentTrajectory; 
                    
                    currentSegmentIdx(uavIdx) = nextSegIdx; 
                    state.hasTransitioned = true; 
                    state.previousSegmentIndex = currentSegIdx; 
                    segmentStates{uavIdx} = state;  % Write back
                    
                    % Update planner reference
                    if ~isempty(planners{uavIdx}) 
                        planners{uavIdx} = updatePlannerReference(planners{uavIdx}, nextSegTraj); 
                        
                        if isfield(planners{uavIdx}, 'refPath') && ~isstruct(planners{uavIdx}.refPath) 
                            planners{uavIdx}.refPath = convertToRefPathStruct(planners{uavIdx}.refPath); 
                        end
                    end
                    
                    % Reset trajectory
                    currentTrajectories{uavIdx} = []; 
                    trajectoryStep(uavIdx) = 0; 
                    stepsSinceReplan(uavIdx) = 0; 
                    
                    % *** Optimization 1: Output only for key events ***
                    fprintf('[t=%.1fs] UAV%d State B->C: Reached segment %d end, switching to segment %d (speed %.2fm/s)\n', ... 
                        time, uavIdx, currentSegIdx, nextSegIdx, currentSpeed); 
                    
                    % Update current segment info
                    currentSegIdx = nextSegIdx; 
                    currentSegTraj = nextSegTraj; 
                    currentSegEnd = nextSegTraj(end, 1:3); 
                    distToCurrentEnd = norm(currentPos - currentSegEnd); 
                end
                
            
            end
            
            % ============================================================
            % 5. Phase C->D: Away from Old Segment, Delete Old Segment
            % ============================================================
            if state.hasTransitioned
                transitionPoint = state.transitionPoint; 
                distFromTransition = norm(currentPos - transitionPoint); 
                
                if distFromTransition > 5.0
                    prevSegIdx = state.previousSegmentIndex; 
                    
                    state.nextSegmentAvailable = false; 
                    state.nextSegmentIndex = []; 
                    state.nextSegmentTrajectory = []; 
                    state.hasTransitioned = false; 
                    state.previousSegmentIndex = []; 
                    state.transitionPoint = []; 
                    segmentStates{uavIdx} = state;  % Write back
                    
                    % *** Optimization 1: Reduce output ***
                    if mod(logCount, OUTPUT_INTERVAL) == 0 || logCount <= 3
                        fprintf('[t=%.1fs] UAV%d State C->D: Deleting segment %d (%.1fm away)\n', ... 
                            time, uavIdx, prevSegIdx, distFromTransition); 
                    end
                end
            end
            
          % ============================================================
% 6. *** Arrival Detection: <20m marks arrival, decelerate to 0 along trajectory ***
% ============================================================
if currentSegIdx >= length(segmentTrajectories{uavIdx}) 
    finalEnd = segmentTrajectories{uavIdx}{end}(end, 1:3); 
    distToFinalEnd = norm(currentPos - finalEnd); 
    currentSpeed = norm(actualState(4:6)); 
    
    % *** Arrival Detection: distance <20m, immediately mark as arrived ***
    if distToFinalEnd < 20.0 && ~isDecelerating(uavIdx) 
        isDecelerating(uavIdx) = true; 
        
        % * Immediately mark as arrived (add here)
        uavArrived(uavIdx) = true; 
        uavArrivalTime(uavIdx) = time; 
        
        % Calculate deceleration direction
        if currentSpeed > 0.1
            decelerationDir(uavIdx, :) = actualState(4:6) / currentSpeed; 
        else
            dirToEnd = finalEnd - currentPos; 
            if norm(dirToEnd) > 0.01
                decelerationDir(uavIdx, :) = dirToEnd / norm(dirToEnd); 
            else
                decelerationDir(uavIdx, :) = [1, 0, 0]; 
            end
        end
        
        fprintf('[t=%.1fs] UAV%d Arrival detected! Distance %.1fm, speed %.2fm/s -> Decelerating along trajectory to 0\n', ... 
            time, uavIdx, distToFinalEnd, currentSpeed); 
    end
    
    % Execute deceleration
    if isDecelerating(uavIdx) 
        deltaV = -accMax * 0.1;  
        newSpeed = max(0, currentSpeed + deltaV); 
        
        newVelocity = decelerationDir(uavIdx, :) * newSpeed; 
        nextMotion = [currentPos, newVelocity, 0,0,0, 1,0,0,0, 0,0,0]; 
        move(egoUAVs{uavIdx}, nextMotion); 
        
        % Complete stop detection
        if newSpeed < 0.01
            segmentCompleted(uavIdx) = true; 
            
            holdMotion = [currentPos, 0,0,0, 0,0,0, 1,0,0,0, 0,0,0]; 
            move(egoUAVs{uavIdx}, holdMotion); 
            
            fprintf('[t=%.1fs] UAV%d Mission complete, stopped ✓ (Final distance to endpoint %.1fm)\n', ... 
                time, uavIdx, norm(currentPos - finalEnd)); 
        end
        
        continue; 
    end
end
            
            % ============================================================
            % 7. Sensor Tracking (Using Cache)
            % ============================================================
            otherUAVs = egoUAVs([1:uavIdx-1, uavIdx+1:end]); 
            
            % *** Optimization 2: Update sensor data at intervals ***
            if mod(logCount, SENSOR_UPDATE_INTERVAL) == 0
                % Radar tracking
                try
                    [~, radarTracks, ~] = updateRadarTracker(sensors{uavIdx}.radar, ... 
                        trackers{uavIdx}.radarPHD, otherUAVs, egoUAVs{uavIdx}, ... 
                        sensors{uavIdx}.radarLoc, sensors{uavIdx}.radarAng, time); 
                    sensorCache(uavIdx).radarTracks = radarTracks;
                catch ME
                    if logCount == 1
                        warning('UAV%d radar tracking failed: %s', uavIdx, ME.message); 
                    end
                    radarTracks = objectTrack.empty;
                end
                
                % LiDAR tracking
                try
                    [~, lidarTracks, ~, ~] = updateLidarTracker(sensors{uavIdx}.lidar, ... 
                        sensors{uavIdx}.lidarDetector, trackers{uavIdx}.lidarJPDA, actualPose, time); 
                    sensorCache(uavIdx).lidarTracks = lidarTracks;
                catch ME
                    if logCount == 1
                        warning('UAV%d LiDAR tracking failed: %s', uavIdx, ME.message); 
                    end
                    lidarTracks = objectTrack.empty;
                end
                
                sensorCache(uavIdx).lastUpdateTime = time;
            else
                % Use cached data
                radarTracks = sensorCache(uavIdx).radarTracks;
                lidarTracks = sensorCache(uavIdx).lidarTracks;
            end
            
            % Fuse tracks
            allTracks = [lidarTracks; radarTracks]; 
            collisionValidators{uavIdx} = updateCollisionValidator(... 
                collisionValidators{uavIdx}, actualState, [], allTracks, time); 
            
           % ============================================================
% 8. MPC Trajectory Planning (Pass Segment End Info + Dynamic Speed Limit)
% ============================================================
if ~isempty(planners{uavIdx}) && ~isempty(currentSegTraj)
    hasCloseObstacles = hasCloseTracks(allTracks, actualState);
    planners{uavIdx}.hasCloseObstacles = hasCloseObstacles;
    
    needReplan = isempty(currentTrajectories{uavIdx}) || ...
                stepsSinceReplan(uavIdx) >= replanSteps || ...
                (hasCloseObstacles && stepsSinceReplan(uavIdx) >= 2);
    
    if needReplan
        % *** Dynamic Speed Limit: Adjust based on distance to endpoint ***
        dynamicSpeedLimit = speedLimit;  % Default 15 m/s
        
        if distToCurrentEnd < 30
            if distToCurrentEnd < 5
                dynamicSpeedLimit = 2.0;
            elseif distToCurrentEnd < 10
                dynamicSpeedLimit = 5.0;
            elseif distToCurrentEnd < 20
                dynamicSpeedLimit = 10.0;
            else
                dynamicSpeedLimit = 12.0;
            end
            
            if mod(logCount, 100) == 0
                fprintf('[t=%.1fs] UAV%d %.1fm from endpoint, speed limit %.1fm/s\n', ...
                    time, uavIdx, distToCurrentEnd, dynamicSpeedLimit);
            end
        end
        
        if logCount <= 3
            fprintf('[t=%.1fs] UAV%d Starting MPC planning...\n', time, uavIdx);
        end
        
        if isfield(planners{uavIdx}, 'refPath') && ...
           isstruct(planners{uavIdx}.refPath)
            
            if logCount <= 3
                fprintf('  planner.refPath: %d points, length %.1fm\n', ...
                    size(planners{uavIdx}.refPath.xyz, 1), ...
                    planners{uavIdx}.refPath.sMax);
            end
            
            % * Pass dynamic speed limit
            candidateTrajectories = generateCandidateTrajectories(...
                planners{uavIdx}, actualState, planners{uavIdx}.refPath, ...
                dynamicSpeedLimit, distToCurrentEnd);
            
            if logCount <= 3
                if ~isempty(candidateTrajectories)
                    fprintf('  Generated %d candidate trajectories ✓\n', length(candidateTrajectories));
                else
                    fprintf('  ⚠ Candidate trajectories empty!\n');
                end
            end
        else
            if logCount <= 3
                fprintf('  [Warning] UAV%d planner.refPath unavailable, temporary conversion\n', uavIdx);
            end
            tempRefPath = convertToRefPathStruct(currentSegTraj);
            
            candidateTrajectories = generateCandidateTrajectories(...
                planners{uavIdx}, actualState, tempRefPath, ...
                dynamicSpeedLimit, distToCurrentEnd);
        end
        
        if ~isempty(candidateTrajectories)
            isKinematicsFeasible = checkKinematicFeasibility(...
                candidateTrajectories, dynamicSpeedLimit, accMax);  % * Use dynamic speed limit
            feasibleTrajectories = candidateTrajectories(isKinematicsFeasible);
            
            if logCount <= 3
                fprintf('  Kinematically feasible: %d/%d trajectories\n', ...
                    sum(isKinematicsFeasible), length(candidateTrajectories));
            end
            
            if ~isempty(feasibleTrajectories)
                % Static obstacle check
                isBuildingCollision = false(length(feasibleTrajectories), 1);
                for trajIdx = 1:length(feasibleTrajectories)
                    traj = feasibleTrajectories{trajIdx};
                    trajPoints = traj(:, 1:3);
                    occStatus = checkOccupancy(omap, trajPoints);
                    isBuildingCollision(trajIdx) = any(occStatus > 0.9);
                end
                
                % Dynamic obstacle check
                [isValidDynamic, collisionProb] = validateTrajectories(...
                    collisionValidators{uavIdx}, feasibleTrajectories);
                
                isCollisionFree = ~isBuildingCollision & isValidDynamic;
                safeTrajectories = feasibleTrajectories(isCollisionFree);
                
                if logCount <= 3
                    fprintf('  Collision-free: %d/%d trajectories (Building: %d, Dynamic: %d)\n', ...
                        sum(isCollisionFree), length(feasibleTrajectories), ...
                        sum(isBuildingCollision), sum(~isValidDynamic));
                end
                
                if ~isempty(safeTrajectories)
                    % * Pass distance to endpoint to cost function
                    costs = calculateTrajectoryCosts(safeTrajectories, ...
                        collisionProb(isCollisionFree), actualState, ...
                        planners{uavIdx}.refPath, [], [], [], distToCurrentEnd);
                    
                    [~, optIdx] = min(costs);
                    fullTrajectory = safeTrajectories{optIdx};
                    controlSteps = min(replanSteps + 1, size(fullTrajectory, 1));
                    currentTrajectories{uavIdx} = fullTrajectory(1:controlSteps, :);
                    trajectoryStep(uavIdx) = 1;
                    previousFullTrajectory{uavIdx} = fullTrajectory;
                    
                    if logCount <= 3
                        fprintf('  ✓ Selected trajectory %d, control steps %d\n', optIdx, controlSteps);
                    end
                else
                    if logCount <= 3
                        fprintf('  ⚠ All trajectories have collisions, using Fallback\n');
                    end
                end
            else
                if logCount <= 3
                    fprintf('  ⚠ No kinematically feasible trajectories, using Fallback\n');
                end
            end
        else
            if logCount <= 3
                fprintf('  ⚠ Candidate trajectory generation failed, using Fallback\n');
            end
        end
    end
end
            
            % ============================================================
            % 9. Trajectory Execution
            % ============================================================
            if ~isempty(currentTrajectories{uavIdx}) && ... 
               trajectoryStep(uavIdx) < size(currentTrajectories{uavIdx}, 1) 
                
                trajectoryStep(uavIdx) = trajectoryStep(uavIdx) + 1; 
                stepsSinceReplan(uavIdx) = stepsSinceReplan(uavIdx) + 1; 
                nextState = currentTrajectories{uavIdx}(trajectoryStep(uavIdx), :); 
                nextMotion = [nextState(1:3), nextState(4:6), 0,0,0, 1,0,0,0, 0,0,0]; 
                move(egoUAVs{uavIdx}, nextMotion); 
            else
                executeSimpleTracking(egoUAVs{uavIdx}, actualState, currentSegTraj, speedLimit); 
            end
            
            % ============================================================
            % 10. Data Recording
            % ============================================================
            currentPointer = systemLog.bufferPointer{uavIdx}; 
            if currentPointer <= MEMORY_CONFIG.maxHistorySteps
                systemLog.states{uavIdx}(currentPointer, :) = actualState; 
            end
            systemLog.bufferPointer{uavIdx} = mod(currentPointer, MEMORY_CONFIG.maxHistorySteps) + 1; 
            
        catch ME
            if logCount == 1
                warning('UAV%d main loop exception: %s', uavIdx, ME.message); 
            end
        end
    end
    
    % *** Optimization 1: Greatly reduce progress output frequency ***
    if mod(logCount, OUTPUT_INTERVAL) == 0
        completedCount = sum(segmentCompleted); 
        activeSegments = zeros(numUAVs, 1); 
        for u = 1:numUAVs
            if ~segmentCompleted(u) 
                activeSegments(u) = currentSegmentIdx(u); 
            end
        end
        fprintf('Progress: %d/%d | Time: %.1fs | Completed: %d/%d | Current Segments: %s\n', ... 
            logCount, maxSteps, time, completedCount, numUAVs, mat2str(activeSegments')); 
    end
    
    % Early termination condition
    if all(segmentCompleted) 
        fprintf('\n✓ All UAVs have completed all trajectory segments, ending simulation early (t=%.1fs)\n', time); 
        break; 
    end
end

fprintf('\nSimulation complete!\n'); 
fprintf('Total Steps: %d | Total Time: %.1fs\n', logCount, scene.CurrentTime);
%% ============================================================
%% 12.x Post-simulation: Actual Trajectory vs Reference Path Visualization
%% ============================================================
fprintf('\n12.x Visualization: Actual Trajectory vs Reference Path...\n'); 

try
    fig_traj = figure('Name', 'UAV Actual Trajectory vs Reference Path', ... 
        'Position', [120, 80, 1200, 800], ... 
        'Color', 'w'); 
    ax_traj = axes('Parent', fig_traj); 
    hold(ax_traj, 'on'); 
    grid(ax_traj, 'on'); 
    axis(ax_traj, 'equal'); 
    xlabel(ax_traj, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    ylabel(ax_traj, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    zlabel(ax_traj, 'Z (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    title(ax_traj, 'UAV Actual Trajectory vs Reference Path', ... 
        'FontSize', 14, 'FontWeight', 'bold'); 
    
    % Define colors
    trajColors = { 
        [1 0 1];      % Magenta
        [0 1 1];      % Cyan
        [1 1 0];      % Yellow
        [1 0.5 0];    % Orange
        [0.5 0 1];    % Purple
        [0 1 0.5];    % Teal
        [1 0.5 0.5];  % Pink
        [0.5 1 0];    % Lime
        [0 0.5 1]     % Blue
    }; 
    
    % Plot each UAV's trajectory
    for u = 1:numUAVs
        colorIdx = mod(u-1, length(trajColors)) + 1; 
        c = trajColors{colorIdx}; 
        
        % Reference path
        if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1
            refPath = optimizedRefPaths{u}; 
            refLabel = sprintf('UAV%d Ref(MinSnap)', u); 
            refStyle = '-.';
        elseif ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1
            refPath = globalRefPaths{u}; 
            refLabel = sprintf('UAV%d Ref(BiRRT)', u); 
            refStyle = '--';
        else
            refPath = []; 
        end
        
        if ~isempty(refPath) 
            % Plot reference path
            plot3(ax_traj, ... 
                refPath(:,1), refPath(:,2), refPath(:,3), ... 
                refStyle, ... 
                'LineWidth', 2.0, ... 
                'Color', c, ... 
                'DisplayName', refLabel); 
            
            % Start point (green)
            scatter3(ax_traj, ... 
                refPath(1,1), refPath(1,2), refPath(1,3), ... 
                60, 'g', 'filled', ... 
                'MarkerEdgeColor', 'k', ... 
                'LineWidth', 1.2, ...
                'HandleVisibility', 'off'); 
            
            % End point (red)
            scatter3(ax_traj, ... 
                refPath(end,1), refPath(end,2), refPath(end,3), ... 
                60, 'r', 'filled', ... 
                'MarkerEdgeColor', 'k', ... 
                'LineWidth', 1.2, ...
                'HandleVisibility', 'off'); 
        end
        
        % Actual flight trajectory
        if exist('systemLog','var') && ~isempty(systemLog) && ... 
           isfield(systemLog, 'states') && u <= numel(systemLog.states) && ... 
           ~isempty(systemLog.states{u}) 
            
            states_u = systemLog.states{u}; 
            validIdx = ~any(isnan(states_u), 2) & ~all(states_u == 0, 2); 
            states_u = states_u(validIdx, :); 
            
            if ~isempty(states_u) && size(states_u,1) > 1
                plot3(ax_traj, ... 
                    states_u(:,1), states_u(:,2), states_u(:,3), ... 
                    '-', ... 
                    'LineWidth', 1.5, ... 
                    'Color', c * 0.65, ... 
                    'DisplayName', sprintf('UAV%d Actual Trajectory', u)); 
            end
        end
    end
    
    % Charging stations
    if exist('charger_pos','var') && ~isempty(charger_pos) 
        scatter3(ax_traj, ... 
            charger_pos(:,1), charger_pos(:,2), charger_pos(:,3), ... 
            70, 'b', 'filled', ... 
            'MarkerEdgeColor', 'k', ... 
            'LineWidth', 1.0, ... 
            'DisplayName', 'Charging Stations'); 
    end
    
    % Task points
    if ~isempty(taskInfo) && isfield(taskInfo, 'tasks') && ... 
       isfield(taskInfo.tasks, 'positions') 
        tasks_pos = taskInfo.tasks.positions; 
        scatter3(ax_traj, ... 
            tasks_pos(:,1), tasks_pos(:,2), tasks_pos(:,3), ... 
            50, '^', 'r', 'filled', ... 
            'MarkerEdgeColor', 'k', ... 
            'LineWidth', 1.0, ... 
            'DisplayName', 'Task Points'); 
    end
    
    % Set axis limits
    if exist('xMin','var') && exist('xMax','var') 
        xlim(ax_traj, [xMin, xMax]); 
    end
    if exist('yMin','var') && exist('yMax','var') 
        ylim(ax_traj, [yMin, yMax]); 
    end
    if exist('zMin','var') && exist('zMax','var') 
        zlim(ax_traj, [zMin, zMax]); 
    end
    
    view(ax_traj, 45, 30); 
    lgd_traj = legend(ax_traj, 'Location', 'northeastoutside'); 
    lgd_traj.FontSize = 9; 
    lgd_traj.Box = 'on'; 
    
    fprintf('  ✓ Actual Trajectory vs Reference Path visualization complete\n'); 
catch ME
    fprintf('  ⚠ Actual trajectory visualization failed: %s\n', ME.message); 
end

%% ============================================================
%% Arrival Statistics
%% ============================================================
fprintf('\n====== UAV Arrival Statistics ======\n'); 
for u = 1:numUAVs
    if ~isempty(optimizedRefPaths{u}) 
        currentRefPath = optimizedRefPaths{u}; 
    elseif ~isempty(globalRefPaths{u}) 
        currentRefPath = globalRefPaths{u}; 
    else
        currentRefPath = []; 
    end
    
    if isempty(currentRefPath) 
        fprintf('UAV%d: No planned path\n', u); 
    elseif uavArrived(u) 
        fprintf('UAV%d: ✓ Arrived (t=%.1fs)\n', u, uavArrivalTime(u)); 
    else
        goalPos = currentRefPath(end, 1:3); 
        states = systemLog.states{u}; 
        validStates = states(~any(isnan(states), 2) & ~all(states == 0, 2), :); 
        
        if ~isempty(validStates) 
            finalPos = validStates(end, 1:3); 
            distToGoal = norm(finalPos - goalPos); 
            fprintf('UAV%d: ✗ Not arrived (%.1fm from goal)\n', u, distToGoal); 
        else
            fprintf('UAV%d: ✗ No valid trajectory\n', u); 
        end
    end
end
fprintf('=========================\n\n'); 

%% ============================================================
%% 13. Save Results
%% ============================================================
fprintf('\n13. Saving results...\n'); 
results = struct(); 
results.assignment = assignment; 
results.uavTasks = cell(numUAVs, 1); 
for u = 1:numUAVs
    if ~isempty(assignment) 
        results.uavTasks{u} = find(assignment == u); 
    end
end
results.globalRefPaths = globalRefPaths; 
results.optimizedRefPaths = optimizedRefPaths; 
results.systemLog = systemLog; 
results.omap = omap; 
results.charger_pos = charger_pos; 
results.taskInfo = taskInfo; 

save('multiUAV_integrated_results.mat', 'results'); 
fprintf('  ✓ Saved to multiUAV_integrated_results.mat\n'); 

%% ============================================================
%% 14. Results Summary and Lightweight Visualization
%% ============================================================
fprintf('\n14. Results Summary and Lightweight Visualization...\n'); 

%% 14.1 Reference Path vs Actual Flight Length Comparison
fprintf('  [1/2] Reference Path vs Actual Flight Statistics...\n'); 
for u = 1:numUAVs
    if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1
        refPath = optimizedRefPaths{u}; 
    elseif ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1
        refPath = globalRefPaths{u}; 
    else
        refPath = []; 
    end
    
    hasPlan = ~isempty(refPath) && size(refPath, 1) > 1; 
    hasActual = false; 
    validStates = []; 
    
    if exist('systemLog', 'var') && ~isempty(systemLog) && ... 
       isfield(systemLog, 'states') && u <= length(systemLog.states) && ... 
       ~isempty(systemLog.states{u}) 
        states = systemLog.states{u}; 
        validIdx = ~any(isnan(states), 2) & ~all(states == 0, 2); 
        validStates = states(validIdx, :); 
        hasActual = ~isempty(validStates) && size(validStates, 1) > 1; 
    end
    
    if hasPlan && hasActual
        planLength = 0; 
        for iSeg = 1:(size(refPath, 1) - 1) 
            planLength = planLength + norm(refPath(iSeg+1, 1:3) - refPath(iSeg, 1:3)); 
        end
        
        actualLength = 0; 
        for iSeg = 1:(size(validStates, 1) - 1) 
            actualLength = actualLength + norm(validStates(iSeg+1, 1:3) - validStates(iSeg, 1:3)); 
        end
        
        deviation = abs(actualLength - planLength); 
        deviationPercent = (deviation / max(planLength, 1e-6)) * 100; 
        
        fprintf('    UAV%d:\n', u); 
        fprintf('      Reference Path: %.1fm (%d points)\n', planLength, size(refPath, 1)); 
        fprintf('      Actual Flight: %.1fm (%d points)\n', actualLength, size(validStates, 1)); 
        fprintf('      Total Length Deviation: %.1fm (%.1f%%)\n', deviation, deviationPercent); 
    elseif hasPlan
        fprintf('    UAV%d: Has reference path, no actual flight data\n', u); 
    else
        fprintf('    UAV%d: No reference path\n', u); 
    end
end

%% 14.2 Actual Trajectory vs Reference Path + OccupancyMap3D
fprintf('\n  [2/2] Actual Trajectory vs Reference Path + OccupancyMap3D City Scene...\n'); 

try
    fig_traj2 = figure('Name', 'UAV Actual Trajectory vs Reference Path + Occupancy Grid City Scene', ... 
        'Position', [140, 100, 1200, 800], ... 
        'Color', 'w'); 
    ax_traj2 = axes('Parent', fig_traj2); 
    hold(ax_traj2, 'on'); 
    grid(ax_traj2, 'on'); 
    axis(ax_traj2, 'equal'); 
    xlabel(ax_traj2, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    ylabel(ax_traj2, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    zlabel(ax_traj2, 'Z (m)', 'FontSize', 12, 'FontWeight', 'bold'); 
    title(ax_traj2, 'UAV Actual Trajectory vs Reference Path (Overlaid on OccupancyMap3D City Scene)', ... 
        'FontSize', 14, 'FontWeight', 'bold'); 
    
    % Display occupancyMap3D
    try
        show(omap, 'Parent', ax_traj2); 
        hold(ax_traj2, 'on'); 
    catch ME_omap
        fprintf('  ⚠ Failed to display occupancyMap3D: %s\n', ME_omap.message); 
    end
    
    % Plot trajectories (same as above)
    for u = 1:numUAVs
        colorIdx = mod(u-1, length(trajColors)) + 1; 
        c = trajColors{colorIdx}; 
        
        if ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1
            refPath = optimizedRefPaths{u}; 
            refLabel = sprintf('UAV%d Ref(MinSnap)', u); 
            refStyle = '-.';
        elseif ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1
            refPath = globalRefPaths{u}; 
            refLabel = sprintf('UAV%d Ref(BiRRT)', u); 
            refStyle = '--';
        else
            refPath = []; 
        end
        
        if ~isempty(refPath) 
            plot3(ax_traj2, ... 
                refPath(:,1), refPath(:,2), refPath(:,3), ... 
                refStyle, 'LineWidth', 2.0, 'Color', c, 'DisplayName', refLabel); 
            scatter3(ax_traj2, refPath(1,1), refPath(1,2), refPath(1,3), ... 
                60, 'g', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
                'HandleVisibility', 'off'); 
            scatter3(ax_traj2, refPath(end,1), refPath(end,2), refPath(end,3), ... 
                60, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
                'HandleVisibility', 'off'); 
        end
        
        if exist('systemLog','var') && ~isempty(systemLog) && ... 
           isfield(systemLog, 'states') && u <= numel(systemLog.states) && ... 
           ~isempty(systemLog.states{u}) 
            states_u = systemLog.states{u}; 
            validIdx = ~any(isnan(states_u), 2) & ~all(states_u == 0, 2); 
            states_u = states_u(validIdx, :); 
            if ~isempty(states_u) && size(states_u,1) > 1
                plot3(ax_traj2, states_u(:,1), states_u(:,2), states_u(:,3), ... 
                    '-', 'LineWidth', 1.5, 'Color', c * 0.65, ... 
                    'DisplayName', sprintf('UAV%d Actual Trajectory', u)); 
            end
        end
    end
    
    % Charging stations and task points
    if exist('charger_pos','var') && ~isempty(charger_pos) 
        scatter3(ax_traj2, charger_pos(:,1), charger_pos(:,2), charger_pos(:,3), ... 
            70, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, ... 
            'DisplayName', 'Charging Stations'); 
    end
    
    if ~isempty(taskInfo) && isfield(taskInfo, 'tasks') && ... 
       isfield(taskInfo.tasks, 'positions') 
        tasks_pos = taskInfo.tasks.positions; 
        scatter3(ax_traj2, tasks_pos(:,1), tasks_pos(:,2), tasks_pos(:,3), ... 
            50, '^', 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0, ... 
            'DisplayName', 'Task Points'); 
    end
    
    if exist('xMin','var') && exist('xMax','var'), xlim(ax_traj2, [xMin, xMax]); end
    if exist('yMin','var') && exist('yMax','var'), ylim(ax_traj2, [yMin, yMax]); end
    if exist('zMin','var') && exist('zMax','var'), zlim(ax_traj2, [zMin, zMax]); end
    
    view(ax_traj2, 45, 30); 
    lgd_traj2 = legend(ax_traj2, 'Location', 'northeastoutside'); 
    lgd_traj2.FontSize = 9; 
    lgd_traj2.Box = 'on'; 
    
    fprintf('  ✓ Actual Trajectory vs Reference Path + OccupancyMap3D visualization complete\n'); 
catch ME
    fprintf('  ⚠ Actual trajectory visualization failed: %s\n', ME.message); 
end

%% 14.3 BiRRT vs Minimum Snap Trajectory Channel Comparison (X, Y, Z)
fprintf('\n  [3/3] BiRRT vs Minimum Snap Trajectory Channel Comparison...\n');

try
    % Find the first UAV that has both BiRRT and Minimum Snap paths
    selectedUAV = 0;
    for u = 1:numUAVs
        hasBiRRT = ~isempty(globalRefPaths{u}) && size(globalRefPaths{u}, 1) > 1;
        hasMinSnap = ~isempty(optimizedRefPaths{u}) && size(optimizedRefPaths{u}, 1) > 1;
        if hasBiRRT && hasMinSnap
            % Check if they are different (i.e., Minimum Snap actually optimized)
            if ~isequal(globalRefPaths{u}, optimizedRefPaths{u})
                selectedUAV = u;
                break;
            end
        end
    end
    
    if selectedUAV > 0
        biRRT_path = globalRefPaths{selectedUAV};
        minSnap_path = optimizedRefPaths{selectedUAV};
        
        % Create normalized parameter (arc length based) for fair comparison
        % BiRRT arc length parameter
        biRRT_arcLen = zeros(size(biRRT_path, 1), 1);
        for i = 2:size(biRRT_path, 1)
            biRRT_arcLen(i) = biRRT_arcLen(i-1) + norm(biRRT_path(i, 1:3) - biRRT_path(i-1, 1:3));
        end
        biRRT_s = biRRT_arcLen / max(biRRT_arcLen(end), 1e-6);  % Normalize to [0, 1]
        
        % Minimum Snap arc length parameter
        minSnap_arcLen = zeros(size(minSnap_path, 1), 1);
        for i = 2:size(minSnap_path, 1)
            minSnap_arcLen(i) = minSnap_arcLen(i-1) + norm(minSnap_path(i, 1:3) - minSnap_path(i-1, 1:3));
        end
        minSnap_s = minSnap_arcLen / max(minSnap_arcLen(end), 1e-6);  % Normalize to [0, 1]
        
        % Create figure with 3x1 subplots
        fig_channel = figure('Name', 'BiRRT vs Minimum Snap Trajectory Comparison (X, Y, Z Channels)', ...
            'Position', [200, 80, 900, 800], ...
            'Color', 'w');
        
        channelNames = {'X', 'Y', 'Z'};
        channelUnits = {'X [meters]', 'Y [meters]', 'Z [meters]'};
        
        for ch = 1:3
            ax_ch = subplot(3, 1, ch);
            hold(ax_ch, 'on');
            grid(ax_ch, 'on');
            box(ax_ch, 'on');
            
            % Plot BiRRT trajectory (dashed line, before smoothing)
            plot(ax_ch, biRRT_s, biRRT_path(:, ch), ...
                '--', 'LineWidth', 2.0, 'Color', [0.8 0.2 0.2], ...
                'DisplayName', 'BiRRT (Before Smoothing)');
            
            % Plot Minimum Snap trajectory (solid line, after smoothing)
            plot(ax_ch, minSnap_s, minSnap_path(:, ch), ...
                '-', 'LineWidth', 2.0, 'Color', [0.2 0.6 0.2], ...
                'DisplayName', 'Minimum Snap (After Smoothing)');
            
            % Labels and formatting
            xlabel(ax_ch, 'Normalized Arc Length s \in [0, 1]', 'FontSize', 11, 'FontWeight', 'bold');
            ylabel(ax_ch, channelUnits{ch}, 'FontSize', 11, 'FontWeight', 'bold');
            title(ax_ch, sprintf('%s Channel: BiRRT vs Minimum Snap (UAV %d)', channelNames{ch}, selectedUAV), ...
                'FontSize', 12, 'FontWeight', 'bold');
            
            % Legend
            if ch == 1
                lgd = legend(ax_ch, 'Location', 'best');
                lgd.FontSize = 10;
                lgd.Box = 'on';
            end
            
            % Set axis limits with some padding
            xlim(ax_ch, [0, 1]);
            yData = [biRRT_path(:, ch); minSnap_path(:, ch)];
            yRange = max(yData) - min(yData);
            if yRange < 1e-6, yRange = 1; end
            ylim(ax_ch, [min(yData) - 0.05*yRange, max(yData) + 0.05*yRange]);
        end
        
        % Add overall title
        sgtitle(fig_channel, 'Trajectory Smoothness Comparison: BiRRT (Global Planning) vs Minimum Snap (Optimized)', ...
            'FontSize', 14, 'FontWeight', 'bold');
        
        fprintf('  ✓ BiRRT vs Minimum Snap channel comparison complete (UAV %d)\n', selectedUAV);
        
        % Calculate and display smoothness metrics
        fprintf('\n  Smoothness Analysis (UAV %d):\n', selectedUAV);
        for ch = 1:3
            % Compute second derivative approximation (curvature indicator)
            if size(biRRT_path, 1) >= 3
                biRRT_d2 = diff(biRRT_path(:, ch), 2);
                biRRT_roughness = sum(biRRT_d2.^2);
            else
                biRRT_roughness = 0;
            end
            
            if size(minSnap_path, 1) >= 3
                minSnap_d2 = diff(minSnap_path(:, ch), 2);
                minSnap_roughness = sum(minSnap_d2.^2);
            else
                minSnap_roughness = 0;
            end
            
            improvement = (biRRT_roughness - minSnap_roughness) / max(biRRT_roughness, 1e-6) * 100;
            fprintf('    %s Channel - BiRRT Roughness: %.2f, MinSnap Roughness: %.2f, Improvement: %.1f%%\n', ...
                channelNames{ch}, biRRT_roughness, minSnap_roughness, improvement);
        end
    else
        fprintf('  ⚠ No UAV found with both BiRRT and Minimum Snap paths for comparison\n');
    end
catch ME_channel
    fprintf('  ⚠ Channel comparison visualization failed: %s\n', ME_channel.message);
end

fprintf('\n✅ All visualizations complete!\n\n');

%% ============================================================
%% Helper Functions (Only local functions specific to main script)
%% ============================================================

function segments = splitTrajectoryByMTSP(fullTrajectory, mtspPoints) 
    segments = {}; 
    if isempty(fullTrajectory) || isempty(mtspPoints) || size(mtspPoints, 1) < 2
        segments{1} = fullTrajectory; 
        return; 
    end
    for i = 1:size(mtspPoints, 1)-1
        startPt = mtspPoints(i, :); 
        endPt = mtspPoints(i+1, :); 
        startIdx = findClosestPointIndex(fullTrajectory(:, 1:3), startPt); 
        endIdx = findClosestPointIndex(fullTrajectory(:, 1:3), endPt); 
        if startIdx < endIdx
            segment = fullTrajectory(startIdx:endIdx, :); 
        elseif startIdx > endIdx
            segment = fullTrajectory(endIdx:startIdx, :); 
        else
            segment = fullTrajectory(startIdx, :); 
        end
        segments{end+1} = segment; %#ok<AGROW>
    end
end

function idx = findClosestPointIndex(trajectory, point) 
    distances = vecnorm(trajectory - point, 2, 2); 
    [~, idx] = min(distances); 
end

function executeSimpleTracking(uavPlatform, currentState, trajectory, maxSpeed) 
    currentPos = currentState(1:3); 
    segmentStart = trajectory(1, 1:3); 
    segmentEnd = trajectory(end, 1:3); 
    
    distToStart = norm(currentPos - segmentStart); 
    distToEnd = norm(currentPos - segmentEnd); 
    
    if distToEnd < 0.5
        nextMotion = [currentPos, 0,0,0, 0,0,0, 1,0,0,0, 0,0,0]; 
        move(uavPlatform, nextMotion); 
        return; 
    end
    
    distances = vecnorm(trajectory(:, 1:3) - currentPos, 2, 2); 
    [~, closestIdx] = min(distances); 
    lookahead = min(15, size(trajectory, 1) - closestIdx); 
    
    if lookahead > 0
        targetIdx = closestIdx + lookahead; 
        targetPos = trajectory(targetIdx, 1:3); 
        direction = targetPos - currentPos; 
        distToTarget = norm(direction); 
        
        if distToTarget > 0.5 
            direction = direction / distToTarget; 
            if distToEnd < 15.0
                desiredSpeed = maxSpeed * max(0.3, distToEnd / 15.0); 
            else
                desiredSpeed = maxSpeed; 
            end
            newVelocity = direction * desiredSpeed; 
            newPosition = currentPos + newVelocity * 0.2; 
            nextMotion = [newPosition, newVelocity, 0,0,0, 1,0,0,0, 0,0,0]; 
        else
            nextMotion = [currentPos, zeros(1,3), 0,0,0, 1,0,0,0, 0,0,0]; 
        end
    else
        nextMotion = [segmentEnd, zeros(1,3), 0,0,0, 1,0,0,0, 0,0,0]; 
    end
    move(uavPlatform, nextMotion); 
end

function hasClose = hasCloseTracks(tracks, currentState, threshold) 
    if nargin < 3, threshold = 15.0; end
    hasClose = false; 
    currentPos = currentState(1:3); 
    for i = 1:length(tracks) 
        if length(tracks(i).State) >= 3
            trackPos = tracks(i).State(1:3)'; 
            if norm(trackPos - currentPos) < threshold
                hasClose = true; 
                return; 
            end
        end
    end
end

function path_out = removeConsecutiveDuplicates(path_in, tol) 
    if nargin < 2, tol = 1e-6; end
    if isempty(path_in) || size(path_in,1) < 2
        path_out = path_in; 
        return; 
    end
    keep = true(size(path_in,1),1); 
    for i = 2:size(path_in,1) 
        if norm(path_in(i,1:3) - path_in(i-1,1:3)) < tol
            keep(i) = false; 
        end
    end
    path_out = path_in(keep,:); 
end

function ptsSeqPerUAV = buildPtsSeqFromMTSP(simResults, charger_pos) 
    numUAVs = numel(simResults.results); 
    ptsSeqPerUAV = cell(numUAVs, 1); 
    for u = 1:numUAVs
        uLog = simResults.results{u}; 
        if isempty(uLog), ptsSeqPerUAV{u} = []; continue; end
        seq = []; 
        if ~isempty(charger_pos) && u <= size(charger_pos,1) 
            seq = [seq; charger_pos(u,:)]; %#ok<AGROW>
        end
        hasTask = false;
        for k = 1:numel(uLog) 
            e = uLog{k}; 
            switch e.type
                case 'to_task', seq = [seq; e.position]; hasTask = true; %#ok<AGROW>
                case {'to_charger', 'return_final'}, seq = [seq; e.to]; %#ok<AGROW>
            end
        end
        if ~hasTask, ptsSeqPerUAV{u} = []; continue; end
        if size(seq,1) > 1
            keep = true(size(seq,1),1); 
            for i = 2:size(seq,1) 
                if norm(seq(i,:) - seq(i-1,:)) < 1e-6, keep(i) = false; end
            end
            seq = seq(keep,:); 
        end
        ptsSeqPerUAV{u} = seq; 
    end
end