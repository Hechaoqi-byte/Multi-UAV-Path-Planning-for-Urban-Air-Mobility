function [assignment, bestFitness, simResults] = MTSP(n_uavs, taskDef, charger_pos, uav_init, opts)
    % Features:
%   - Full 3D environment support (considering Z-axis height)
%   - Manhattan distance + obstacle penalty (manhattan_obstacles)
%   - Genetic algorithm optimization for task assignment
%   - Endurance time limit (max_endurance) + range/battery (maxRange)
%   - Return to nearest charger when current battery >20%% but would drop below 20%% after next task
%   - Return to nearest charger from final position after all tasks completed
%
% Input:
%   n_uavs      - Number of UAVs
%   taskDef     - Task definition structure
%   charger_pos - Charger positions matrix (M×3, independent of n_uavs)
%   uav_init    - Reserved interface
%   opts        - Configuration structure, key fields:
%                 .omap            - occupancyMap3D for obstacle detection
%                 .distanceMetric  - Distance metric, default 'manhattan_obstacles'
%                 .verticalPenalty - Vertical penalty coefficient
%                 .velocity        - Velocity (m/s)
%                 .t_c             - Charging time (s)
%                 .max_endurance   - Maximum endurance time (s)
%                 .maxRange        - Full battery range (m), default 2000
%                 .PopulationSize, .Generations, q1,q2,q3 etc.
%
% Output:
%   assignment  - N×1 task assignment vector (values are UAV numbers 1~m)
%   bestFitness - Optimal objective function value
%   simResults  - Detailed simulation results structure


    fprintf('\n====================================================\n');
    fprintf('MTSP - Multi-UAV Task Assignment Optimizer\n');
    fprintf('Version: v2.1 (3D + Manhattan + Obstacles + Range/Battery)\n');
    fprintf('====================================================\n\n');
    
    
    %% ================================================================
    %% 1. Parameter Extraction and Default Values
    %% ================================================================
    if nargin < 5
        opts = struct();
    end
    
    % Flight parameters
    if ~isfield(opts, 'velocity')
        opts.velocity = 15;
    end
    if ~isfield(opts, 't_c')
        opts.t_c = 300;
    end
    if ~isfield(opts, 'max_endurance')
        opts.max_endurance = 1800;   % Maximum continuous flight time (seconds), keep original logic
    end
    % Range/Battery (full battery range, meters)
    if ~isfield(opts, 'maxRange')
        opts.maxRange = 2000;        % e.g., 2km range
    end
    
    % GA parameters
    if ~isfield(opts, 'PopulationSize')
        opts.PopulationSize = 50;
    end
    if ~isfield(opts, 'Generations')
        opts.Generations = 100;
    end
    
    % Objective weights
    if ~isfield(opts, 'q1')
        opts.q1 = 0.3;
    end
    if ~isfield(opts, 'q2')
        opts.q2 = 0.4;
    end
    if ~isfield(opts, 'q3')
        opts.q3 = 0.3;
    end
    
    % Distance metric parameters
    if ~isfield(opts, 'distanceMetric')
        opts.distanceMetric = 'manhattan_obstacles';  % Default
    end
    if ~isfield(opts, 'verticalPenalty')
        opts.verticalPenalty = 1.2;
    end
    
    % Check omap
    if strcmp(opts.distanceMetric, 'manhattan_obstacles')
        if ~isfield(opts, 'omap') || isempty(opts.omap)
            warning('manhattan_obstacles requires omap, falling back to manhattan3d');
            opts.distanceMetric = 'manhattan3d';
        end
    end
    
    
    %% ================================================================
    %% 2. Input Validation
    %% ================================================================
    fprintf('Input Data Validation:\n');
    
    % Validate task definition
    if ~isfield(taskDef, 'positions') || isempty(taskDef.positions)
        error('MTSP:InvalidInput', 'taskDef.positions cannot be empty');
    end
    
    events_pos = taskDef.positions;
    
    % Validate 3D coordinates
    fprintf('  Task positions: %d × %d\n', size(events_pos, 1), size(events_pos, 2));
    if size(events_pos, 2) < 3
        error('MTSP:InvalidInput', 'Task coordinates must be 3D (expected N×3, got %d×%d)', ...
            size(events_pos, 1), size(events_pos, 2));
    elseif size(events_pos, 2) > 3
        fprintf('  Warning: Using only first 3 columns\n');
        events_pos = events_pos(:, 1:3);
    end
    
    fprintf('  Charger positions: %d × %d\n', size(charger_pos, 1), size(charger_pos, 2));
    if size(charger_pos, 2) < 3
        error('MTSP:InvalidInput', 'Charger coordinates must be 3D (expected M×3, got %d×%d)', ...
            size(charger_pos, 1), size(charger_pos, 2));
    elseif size(charger_pos, 2) > 3
        fprintf('  Warning: Using only first 3 columns\n');
        charger_pos = charger_pos(:, 1:3);
    end
    
    % Validate altitude
    task_z = [min(events_pos(:,3)), max(events_pos(:,3))];
    charger_z = [min(charger_pos(:,3)), max(charger_pos(:,3))];
    fprintf('  Task altitude: [%.1fm, %.1fm]\n', task_z);
    fprintf('  Charger altitude: [%.1fm, %.1fm]\n', charger_z);
    fprintf('✓ 3D coordinate validation passed\n\n');
    
    % Task durations
    if isfield(taskDef, 'durations') && ~isempty(taskDef.durations)
        events_duration = taskDef.durations;
    else
        events_duration = 60 * ones(size(events_pos, 1), 1);
    end
    
    % Time offsets
    if isfield(taskDef, 'time_offsets') && ~isempty(taskDef.time_offsets)
        time_offsets = taskDef.time_offsets;
    elseif isfield(taskDef, 'times') && ~isempty(taskDef.times)
        if isdatetime(taskDef.times)
            t0 = taskDef.times(1);
            time_offsets = seconds(taskDef.times - t0);
        else
            time_offsets = zeros(size(events_pos, 1), 1);
        end
    else
        time_offsets = zeros(size(events_pos, 1), 1);
    end
    
    
    %% ================================================================
    %% 3. Parameter Summary
    %% ================================================================
    m = n_uavs;
    n = size(events_pos, 1);
    v = opts.velocity;
    t_c = opts.t_c;
    max_endurance = opts.max_endurance;   % Time endurance constraint
    maxRange = opts.maxRange;            % Range/battery constraint
    q1 = opts.q1;
    q2 = opts.q2;
    q3 = opts.q3;
    
    fprintf('MTSP Parameters:\n');
    fprintf('  UAV count: %d\n', m);
    fprintf('  Task count: %d\n', n);
    fprintf('  Flight velocity: %.1f m/s\n', v);
    fprintf('  Charging time: %.0fs\n', t_c);
    fprintf('  Max endurance time: %.0fs (%.1fmin)\n', max_endurance, max_endurance/60);
    fprintf('  Max range (battery): %.1fm\n', maxRange);
    fprintf('  Distance metric: %s\n', opts.distanceMetric);
    if strcmp(opts.distanceMetric, 'manhattan_obstacles')
        fprintf('  Obstacle detection: Enabled\n');
    end
    fprintf('  Vertical penalty: %.2f\n', opts.verticalPenalty);
    fprintf('  Weights: q1=%.2f, q2=%.2f, q3=%.2f\n\n', q1, q2, q3);
    
    
    %% ================================================================
    %% 4. Calculate 3D Distance Matrix (using calculate3DDistance and specified metric)
    %% ================================================================
    fprintf('====================================================\n');
    fprintf('Calculating 3D Distance Matrix (%s)\n', opts.distanceMetric);
    fprintf('====================================================\n');
    
    tic;
    
    if isfield(opts, 'omap')
        omap_for_dist = opts.omap;
    else
        omap_for_dist = [];
    end
    
    % Task-to-task distances
    fprintf('  [1/3] Task-to-task distances (%d×%d)...\n', n, n);
    dist_task_to_task = zeros(n, n);
    for i = 1:n
        for j = 1:n
            if i ~= j
                dist_task_to_task(i,j) = calculate3DDistance(...
                    events_pos(i,:), events_pos(j,:), ...
                    opts.distanceMetric, opts.verticalPenalty, omap_for_dist);
            end
        end
        if mod(i, max(1, floor(n/5))) == 0
            fprintf('      Progress: %d/%d (%.1f%%)\n', i, n, i/n*100);
        end
    end
    
    % Charger-to-task distances
    fprintf('  [2/3] Charger-to-task distances (%d×%d)...\n', m, n);
    dist_charger_to_task = zeros(m, n);
    for i = 1:m
        for j = 1:n
            dist_charger_to_task(i,j) = calculate3DDistance(...
                charger_pos(i,:), events_pos(j,:), ...
                opts.distanceMetric, opts.verticalPenalty, omap_for_dist);
        end
    end
    
    % Charger-to-charger distances
    fprintf('  [3/3] Charger-to-charger distances (%d×%d)...\n', m, m);
    dist_charger_to_charger = zeros(m, m);
    for i = 1:m
        for j = 1:m
            if i ~= j
                dist_charger_to_charger(i,j) = calculate3DDistance(...
                    charger_pos(i,:), charger_pos(j,:), ...
                    opts.distanceMetric, opts.verticalPenalty, omap_for_dist);
            end
        end
    end
    
    distTime = toc;
    
    fprintf('✓ Distance matrix completed (time: %.1fs)\n', distTime);
    fprintf('  Average task-to-task distance: %.1fm\n', mean(dist_task_to_task(dist_task_to_task>0)));
    fprintf('  Maximum task-to-task distance: %.1fm\n', max(dist_task_to_task(:)));
    fprintf('  Average charger-to-task distance: %.1fm\n\n', mean(dist_charger_to_task(:)));
    
    % Time matrices
    time_task_to_task = dist_task_to_task / v;
    time_charger_to_task = dist_charger_to_task / v;
    time_charger_to_charger = dist_charger_to_charger / v;
    
    
    %% ================================================================
    %% 5. Genetic Algorithm Optimization (internal fitness uses range/battery + 20%% logic)
    %% ================================================================
    fprintf('====================================================\n');
    fprintf('Genetic Algorithm Optimization\n');
    fprintf('====================================================\n');
    fprintf('  Population: %d, Generations: %d\n', opts.PopulationSize, opts.Generations);
    fprintf('  Search space: [1,%d]^%d\n\n', m, n);
    
    nvars = n;
    lb = ones(1, n);
    ub = m * ones(1, n);
    intcon = 1:nvars;
    
    gaOpts = optimoptions('ga', ...
        'PopulationSize', opts.PopulationSize, ...
        'MaxGenerations', opts.Generations, ...
        'FunctionTolerance', 1e-4, ...
        'Display', 'iter', ...
        'UseParallel', false);
    
    % Fitness function
    fitnessHandle = @(x) computeFitness_MTSP(x, m, n, ...
        events_pos, charger_pos, events_duration, ...
        dist_charger_to_task, dist_task_to_task, dist_charger_to_charger, ...
        time_charger_to_task, time_task_to_task, time_charger_to_charger, ...
        t_c, max_endurance, q1, q2, q3, time_offsets, v, maxRange, ...
        opts.distanceMetric, opts.verticalPenalty, omap_for_dist);
    
    fprintf('Starting optimization...\n\n');
    gaStart = tic;
    
    try
        [xbest, fbest] = ga(fitnessHandle, nvars, [], [], [], [], ...
            lb, ub, [], intcon, gaOpts);
    catch
        fprintf('Warning: Full GA failed, simplified retry\n');
        try
            [xbest, fbest] = ga(fitnessHandle, nvars, [], [], [], [], ...
                lb, ub, [], gaOpts);
        catch
            fprintf('Error: GA failed, using random solution\n');
            xbest = randi([1,m], 1, n);
            fbest = fitnessHandle(xbest);
        end
    end
    
    gaTime = toc(gaStart);
    
    assignment = round(xbest(:));
    assignment = max(1, min(m, assignment));
    bestFitness = fbest;
    
    fprintf('\n====================================================\n');
    fprintf('Optimization Completed\n');
    fprintf('====================================================\n');
    fprintf('  Time elapsed: %.1fs\n', gaTime);
    fprintf('  Best fitness: %.6f\n\n', bestFitness);
    
    
    %% ================================================================
    %% 6. Task Execution Simulation (same battery logic as fitness)
    %% ================================================================
    fprintf('====================================================\n');
    fprintf('Task Execution Simulation\n');
    fprintf('====================================================\n');
    
    [results, total_distance, max_completion_time, used_chargers, detailed_timeline] = ...
        simulateMission(assignment, m, n, ...
            events_pos, charger_pos, events_duration, ...
            dist_charger_to_task, dist_task_to_task, dist_charger_to_charger, ...
            time_charger_to_task, time_task_to_task, time_charger_to_charger, ...
            t_c, max_endurance, time_offsets, v, ...
            opts.distanceMetric, opts.verticalPenalty, omap_for_dist, maxRange);
    
    
    %% ================================================================
    %% 7. Results Organization
    %% ================================================================
    simResults = struct();
    simResults.results = results;
    simResults.total_distance = total_distance;
    simResults.max_completion_time = max_completion_time;
    simResults.used_chargers = used_chargers;
    simResults.detailed_timeline = detailed_timeline;
    simResults.distance_matrix = struct(...
        'task_to_task', dist_task_to_task, ...
        'charger_to_task', dist_charger_to_task, ...
        'charger_to_charger', dist_charger_to_charger);
    simResults.computation_time = struct(...
        'distance', distTime, ...
        'optimization', gaTime, ...
        'total', distTime + gaTime);
    
    
    %% ================================================================
    %% 8. Summary Output (original output preserved, added range logic statistics)
    %% ================================================================
    fprintf('\n====================================================\n');
    fprintf('Task Assignment Scheme (excluding mid-mission return details)\n');
    fprintf('====================================================\n');
    for i = 1:n
        fprintf('  Task%2d → UAV%d | [%7.1f, %7.1f, %6.1f]\n', ...
            i, assignment(i), events_pos(i,:));
    end
    
    fprintf('\n====================================================\n');
    fprintf('UAV Execution Details (total time/distance includes mid-mission returns)\n');
    fprintf('====================================================\n');
    for i = 1:m
        if isempty(detailed_timeline{i})
            fprintf('\n● UAV%d: No tasks\n', i);
        else
            tl = detailed_timeline{i};
            fprintf('\n● UAV%d:\n', i);
            fprintf('  Assigned tasks: %s (%d tasks)\n', mat2str(tl.tasks'), length(tl.tasks));
            fprintf('  Actual total distance: %.1fm (%.2fkm)\n', tl.total_distance, tl.total_distance/1000);
            fprintf('  Actual total time: %.1fs (%.1fmin)\n', tl.total_time, tl.total_time/60);
        end
    end
    
    fprintf('\n====================================================\n');
    fprintf('System Overall Metrics\n');
    fprintf('====================================================\n');
    fprintf('  Total distance: %.1fm (%.2fkm)\n', total_distance, total_distance/1000);
    fprintf('  Maximum time: %.1fs (%.1fmin)\n', max_completion_time, max_completion_time/60);
    fprintf('  Used chargers (UAVs): %d/%d (%.0f%%)\n', ...
        length(used_chargers), m, length(used_chargers)/m*100);
    fprintf('  Average per UAV: %.1fm\n', total_distance/m);
    fprintf('  Average per task: %.1fm\n', total_distance/n);
    fprintf('  Computation time: %.1fs (distance %.1fs + optimization %.1fs)\n', ...
        distTime+gaTime, distTime, gaTime);
    fprintf('====================================================\n\n');
    
    fprintf('✓ MTSP optimization completed\n\n');
    
end


%% ====================================================================
%% Helper Function: Calculate 3D Distance (supports manhattan_obstacles)
%% ====================================================================
function dist = calculate3DDistance(pos1, pos2, metric, verticalPenalty, omap)
    
    if nargin < 3, metric = 'euclidean3d'; end
    if nargin < 4, verticalPenalty = 1.0; end
    if nargin < 5, omap = []; end
    
    if length(pos1) < 3 || length(pos2) < 3
        error('Coordinates must be 3D');
    end
    
    dx = pos2(1) - pos1(1);
    dy = pos2(2) - pos1(2);
    dz = pos2(3) - pos1(3);
    
    switch metric
        case 'euclidean3d'
            dist = sqrt(dx^2 + dy^2 + dz^2);
            
        case 'manhattan3d'
            dist = abs(dx) + abs(dy) + abs(dz);
            
        case 'weighted3d'
            dist = sqrt(dx^2 + dy^2) + abs(dz) * verticalPenalty;
            
        case 'manhattan_obstacles'
            % Manhattan + obstacles
            base = abs(dx) + abs(dy) + abs(dz) * verticalPenalty;
            
            if isempty(omap)
                dist = base;
                return;
            end
            
            % Sample for obstacle checking
            nSamples = 20;
            obstCount = 0;
            validCount = 0;
            
            for i = 1:nSamples
                alpha = i / nSamples;
                sample = pos1 + alpha * (pos2 - pos1);
                try
                    occ = checkOccupancy(omap, sample);
                    validCount = validCount + 1;
                    if occ > 0.5
                        obstCount = obstCount + 1;
                    end
                catch
                    obstCount = obstCount + 0.5;
                    validCount = validCount + 1;
                end
            end
            
            if validCount > 0
                penalty = 1.0 + (obstCount / validCount) * 0.5;
            else
                penalty = 1.0;
            end
            
            dist = base * penalty;
            
        otherwise
            dist = sqrt(dx^2 + dy^2 + dz^2);
    end
end


%% ====================================================================
%% Helper Function: Find Nearest Charger (for return trips)
%% ====================================================================
function [idx, distMin] = findNearestCharger(pos, charger_pos)
    diffs = charger_pos - pos;
    dists = sqrt(sum(diffs.^2, 2));   % Use Euclidean only for "who is closest", not for energy calculation
    [distMin, idx] = min(dists);
end


%% ====================================================================
%% Fitness Function: Integrated range/battery + 20%% threshold + nearest charger return
%% ====================================================================
function fitness = computeFitness_MTSP(x, m, n, ...
    events_pos, charger_pos, events_duration, ...
    dist_charger_to_task, dist_task_to_task, dist_charger_to_charger, ...
    time_charger_to_task, time_task_to_task, time_charger_to_charger, ...
    t_c, max_endurance, q1, q2, q3, time_offsets, v, maxRange, ...
    metric, vPenalty, omap)
    
    x = round(x);
    x = max(1, min(m, x));
    
    try
        total_dist = 0;
        max_time   = 0;
        num_used   = 0;
        
        for i = 1:m
            tasks = find(x == i);
            if isempty(tasks), continue; end
            
            num_used = num_used + 1;
            [~, idx] = sort(time_offsets(tasks));
            tasks = tasks(idx);
            
            curr_time = 0;
            uav_dist  = 0;
            
            remRange  = maxRange;
            thrRange  = 0.2 * maxRange;
            currPos   = charger_pos(i,:);
            
            for t = 1:length(tasks)
                tid = tasks(t);
                taskPos = events_pos(tid,:);
                
                % currPos → task point
                if t == 1
                    seg_dist = dist_charger_to_task(i, tid);
                    seg_time = time_charger_to_task(i, tid);
                else
                    prevTid  = tasks(t-1);
                    seg_dist = dist_task_to_task(prevTid, tid);
                    seg_time = time_task_to_task(prevTid, tid);
                end
                
                % Insufficient range, return to nearest charger first
                if remRange < seg_dist
                    [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                    back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                    back_time = back_dist / v;
                    
                    uav_dist  = uav_dist + back_dist;
                    curr_time = curr_time + back_time + t_c;
                    remRange  = maxRange;
                    currPos   = charger_pos(nearestIdx,:);
                    
                    seg_dist = calculate3DDistance(currPos, taskPos, metric, vPenalty, omap);
                    seg_time = seg_dist / v;
                end
                
                % Fly to task point
                uav_dist  = uav_dist + seg_dist;
                curr_time = curr_time + seg_time;
                remRange  = remRange - seg_dist;
                currPos   = taskPos;
                
                % Execute task (only consumes time)
                curr_time = curr_time + events_duration(tid);
                
                % 20%% threshold: check next task
                if t < length(tasks)
                    nextTid  = tasks(t+1);
                    nextPos  = events_pos(nextTid,:);
                    distNext = calculate3DDistance(currPos, nextPos, metric, vPenalty, omap);
                    
                    batteryNow   = remRange;
                    batteryAfter = remRange - distNext;
                    
                    if (batteryNow > thrRange) && (batteryAfter < thrRange)
                        [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                        back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                        back_time = back_dist / v;
                        
                        uav_dist  = uav_dist + back_dist;
                        curr_time = curr_time + back_time + t_c;
                        remRange  = maxRange;
                        currPos   = charger_pos(nearestIdx,:);
                    end
                end
                
                % Time endurance constraint
                if curr_time > max_endurance
                    [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                    back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                    back_time = back_dist / v;
                    
                    uav_dist  = uav_dist + back_dist;
                    curr_time = curr_time + back_time + t_c;
                    remRange  = maxRange;
                    currPos   = charger_pos(nearestIdx,:);
                end
            end
            
            % After all tasks completed: return to nearest charger
            [nearestIdxFinal, ~] = findNearestCharger(currPos, charger_pos);
            back_distFinal = calculate3DDistance(currPos, charger_pos(nearestIdxFinal,:), metric, vPenalty, omap);
            back_timeFinal = back_distFinal / v;
            uav_dist       = uav_dist + back_distFinal;
            curr_time      = curr_time + back_timeFinal;
            
            total_dist = total_dist + uav_dist;
            max_time   = max(max_time, curr_time);
        end
        
        J1 = num_used / max(1, m);
        J2 = total_dist / max(1, n * 1000);
        J3 = max_time   / max(1, 3600);
        
        fitness = q1*J1 + q2*J2 + q3*J3;
        
        if isnan(fitness) || isinf(fitness) || fitness < 0
            fitness = 1e6;
        end
        
    catch
        fitness = 1e6;
    end
end


%% ====================================================================
%% Task Execution Simulation: Same logic as fitness, for detailed results
%% ====================================================================
function [results, total_distance, max_completion_time, used_chargers, detailed_timeline] = ...
    simulateMission(assignment, m, n, ...
        events_pos, charger_pos, events_duration, ...
        dist_charger_to_task, dist_task_to_task, dist_charger_to_charger, ...
        time_charger_to_task, time_task_to_task, time_charger_to_charger, ...
        t_c, max_endurance, time_offsets, v, ...
        metric, vPenalty, omap, maxRange)
    
    results = cell(m, 1);
    total_distance = 0;
    max_completion_time = 0;
    used_chargers = [];
    detailed_timeline = cell(m,1);
    
    thrRange = 0.2 * maxRange;
    
    for i = 1:m
        tasks = find(assignment == i);
        if isempty(tasks)
            results{i} = [];
            detailed_timeline{i} = [];
            continue;
        end
        
        used_chargers = [used_chargers, i];
        [~, idx] = sort(time_offsets(tasks));
        tasks = tasks(idx);
        
        curr_time = 0;
        uav_dist  = 0;
        remRange  = maxRange;
        currPos   = charger_pos(i,:);
        
        log = {};   % Record actions
        
        for t = 1:length(tasks)
            tid = tasks(t);
            taskPos = events_pos(tid,:);
            
            % currPos → task point
            if t == 1
                seg_dist = dist_charger_to_task(i, tid);
                seg_time = time_charger_to_task(i, tid);
            else
                prevTid  = tasks(t-1);
                seg_dist = dist_task_to_task(prevTid, tid);
                seg_time = time_task_to_task(prevTid, tid);
            end
            
            % Insufficient range, return to nearest charger first
            if remRange < seg_dist
                [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                back_time = back_dist / v;
                
                uav_dist  = uav_dist + back_dist;
                curr_time = curr_time + back_time;
                log{end+1} = struct('type','to_charger',...
                                    'from',currPos,...
                                    'to',charger_pos(nearestIdx,:),...
                                    'dist',back_dist,...
                                    'time',curr_time);
                
                curr_time = curr_time + t_c;
                log{end+1} = struct('type','charging',...
                                    'at',charger_pos(nearestIdx,:),...
                                    'time',curr_time,...
                                    'duration',t_c);
                
                remRange = maxRange;
                currPos  = charger_pos(nearestIdx,:);
                
                seg_dist = calculate3DDistance(currPos, taskPos, metric, vPenalty, omap);
                seg_time = seg_dist / v;
            end
            
            % Fly to task point
            uav_dist  = uav_dist + seg_dist;
            curr_time = curr_time + seg_time;
            remRange  = remRange - seg_dist;
            currPos   = taskPos;
            log{end+1} = struct('type','to_task',...
                                'task_id',tid,...
                                'position',taskPos,...
                                'dist',seg_dist,...
                                'time',curr_time);
            
            % Execute task
            curr_time = curr_time + events_duration(tid);
            log{end+1} = struct('type','task',...
                                'task_id',tid,...
                                'position',taskPos,...
                                'duration',events_duration(tid),...
                                'time',curr_time);
            
            % 20%% threshold: check next task
            if t < length(tasks)
                nextTid  = tasks(t+1);
                nextPos  = events_pos(nextTid,:);
                distNext = calculate3DDistance(currPos, nextPos, metric, vPenalty, omap);
                
                batteryNow   = remRange;
                batteryAfter = remRange - distNext;
                
                if (batteryNow > thrRange) && (batteryAfter < thrRange)
                    [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                    back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                    back_time = back_dist / v;
                    
                    uav_dist  = uav_dist + back_dist;
                    curr_time = curr_time + back_time;
                    log{end+1} = struct('type','to_charger',...
                                        'from',currPos,...
                                        'to',charger_pos(nearestIdx,:),...
                                        'dist',back_dist,...
                                        'time',curr_time);
                    
                    curr_time = curr_time + t_c;
                    log{end+1} = struct('type','charging',...
                                        'at',charger_pos(nearestIdx,:),...
                                        'time',curr_time,...
                                        'duration',t_c);
                    
                    remRange = maxRange;
                    currPos  = charger_pos(nearestIdx,:);
                end
            end
            
            % Time endurance constraint
            if curr_time > max_endurance
                [nearestIdx, ~] = findNearestCharger(currPos, charger_pos);
                back_dist = calculate3DDistance(currPos, charger_pos(nearestIdx,:), metric, vPenalty, omap);
                back_time = back_dist / v;
                
                uav_dist  = uav_dist + back_dist;
                curr_time = curr_time + back_time;
                log{end+1} = struct('type','to_charger',...
                                    'from',currPos,...
                                    'to',charger_pos(nearestIdx,:),...
                                    'dist',back_dist,...
                                    'time',curr_time);
                
                curr_time = curr_time + t_c;
                log{end+1} = struct('type','charging',...
                                    'at',charger_pos(nearestIdx,:),...
                                    'time',curr_time,...
                                    'duration',t_c);
                
                remRange = maxRange;
                currPos  = charger_pos(nearestIdx,:);
            end
        end
        
        % After all tasks completed: return to nearest charger
        [nearestIdxFinal, ~] = findNearestCharger(currPos, charger_pos);
        back_distFinal = calculate3DDistance(currPos, charger_pos(nearestIdxFinal,:), metric, vPenalty, omap);
        back_timeFinal = back_distFinal / v;
        
        uav_dist  = uav_dist + back_distFinal;
        curr_time = curr_time + back_timeFinal;
        log{end+1} = struct('type','return_final',...
                            'from',currPos,...
                            'to',charger_pos(nearestIdxFinal,:),...
                            'dist',back_distFinal,...
                            'time',curr_time);
        
        total_distance = total_distance + uav_dist;
        max_completion_time = max(max_completion_time, curr_time);
        
        results{i} = log;
        detailed_timeline{i} = struct('uav_id',i,...
                                      'tasks',tasks,...
                                      'total_time',curr_time,...
                                      'total_distance',uav_dist);
    end
end