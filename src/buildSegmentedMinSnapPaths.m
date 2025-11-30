function optimizedRefPaths = buildSegmentedMinSnapPaths( ...
    globalRefPaths, segmentRefPaths, mtspPtsSeq, CONFIG, omap)
% Build segmented Minimum Snap paths based on MTSP point sequence endpoints
%
% Inputs:
%   globalRefPaths  : cell{numUAVs}, each UAV's BiRRT global polyline path [N×3] (for fallback/check)
%   segmentRefPaths : cell{numUAVs}, each UAV's segmented BiRRT paths, segmentRefPaths{u}{seg} = [Ni×3]
%   mtspPtsSeq      : cell{numUAVs}, each UAV's point sequence in execution order
%                     (start charging station → task points / intermediate charging stations → final charging station)
%   CONFIG          : global configuration structure (uses speed/acceleration, minimum_snap config)
%   omap            : occupancyMap3D, for safety checking (can be empty)
%
% Outputs:
%   optimizedRefPaths : cell{numUAVs}, each UAV's global path after segmented Minimum Snap smoothing

numUAVs = numel(globalRefPaths);
optimizedRefPaths = cell(numUAVs, 1);

% ===== Global MinSnap Segment Parameters =====
params = struct();
params.order = 7;                         % Trajectory polynomial order
params.k_r   = 2;                         % Position constraint up to second derivative (velocity/acceleration continuity)
params.k_psi = 0;
params.mu_r  = 1.0;
params.mu_psi = 0.0;
params.corridor_enabled = CONFIG.minimum_snap.corridor_enabled;
params.corridor_width   = CONFIG.minimum_snap.corridor_width;
params.n_intermediate   = CONFIG.minimum_snap.n_intermediate;
params.speed_limit      = CONFIG.speed_limit;
params.acc_max          = CONFIG.acc_max;
params.jerk_max         = 15.0;

fprintf('\n[Segmented MinSnap] Performing Minimum Snap smoothing using mtspPtsSeq segment endpoints...\n');

for u = 1:numUAVs
    fprintf('  [UAV%d] ', u);

    rawGlobal = globalRefPaths{u};

    % Basic check: BiRRT path is sufficiently long
    if isempty(rawGlobal) || size(rawGlobal,1) < 3
        fprintf('BiRRT path insufficient, keeping original\n');
        optimizedRefPaths{u} = rawGlobal;
        continue;
    end

    % Get segment endpoint sequence for this UAV from mtspPtsSeq
    if nargin < 3 || isempty(mtspPtsSeq) || numel(mtspPtsSeq) < u || isempty(mtspPtsSeq{u})
        fprintf('mtspPtsSeq{%d} is empty, keeping BiRRT\n', u);
        optimizedRefPaths{u} = rawGlobal;
        continue;
    end

    ptsSeq = mtspPtsSeq{u};    % [K×3]
    if size(ptsSeq,2) ~= 3
        fprintf('ptsSeq{%d} dimension abnormal, keeping BiRRT\n', u);
        optimizedRefPaths{u} = rawGlobal;
        continue;
    end

    numNodes = size(ptsSeq, 1);   % Number of segment endpoints
    if numNodes < 2
        fprintf('Insufficient segment endpoints, keeping BiRRT\n');
        optimizedRefPaths{u} = rawGlobal;
        continue;
    end

    fprintf('Using mtspPtsSeq as segment endpoints, total %d points, %d segments\n', numNodes, numNodes-1);

    fullPath   = [];
    curr_state = [];

    for seg = 1:(numNodes - 1)
        sp = ptsSeq(seg,   :);
        gp = ptsSeq(seg+1, :);

        % Start and end points almost coincide (e.g., same charging station), skip this segment
        if norm(gp - sp) < 1e-3
            fprintf('    [Segment%02d] Start/end points coincide (%.3fm), skipping\n', seg, norm(gp-sp));
            continue;
        end

        fprintf('\n    [Segment%02d] [%.1f,%.1f,%.1f] → [%.1f,%.1f,%.1f]\n', ...
            seg, sp, gp);

        % 1) Use BiRRT segment path stored during BiRRT phase as raw sub-path
        rawSeg = [];
        if u <= numel(segmentRefPaths) && ~isempty(segmentRefPaths{u}) && ...
           seg <= numel(segmentRefPaths{u}) && ~isempty(segmentRefPaths{u}{seg})
            rawSeg = segmentRefPaths{u}{seg};
            fprintf('      Using BiRRT segment%d as raw path (%d points)\n', ...
                seg, size(rawSeg,1));
        end

        % Fallback to straight line if segment not found (should not normally happen)
        if isempty(rawSeg) || size(rawSeg,1) < 2
            dist = norm(gp - sp);
            numPts = max(5, ceil(dist / 10));
            rawSeg = sp + (gp - sp) .* linspace(0, 1, numPts)';
            fprintf('      ⚠ BiRRT segment%d not found, using linear interpolation (%d points)\n', ...
                seg, size(rawSeg,1));
        end

        % ===== 2) Adaptive key point selection on BiRRT segment =====
        base_max_wp = CONFIG.minimum_snap.max_waypoints;  % Baseline upper limit from config

        % Calculate total length of this BiRRT segment
        seg_len = 0;
        for iLen = 1:(size(rawSeg,1)-1)
            seg_len = seg_len + norm(rawSeg(iLen+1,1:3) - rawSeg(iLen,1:3));
        end

        % Rough estimate of total turning angle (curvature measure)
        turn_measure = 0;
        for iCurv = 2:(size(rawSeg,1)-1)
            v1 = rawSeg(iCurv,:)   - rawSeg(iCurv-1,:);
            v2 = rawSeg(iCurv+1,:) - rawSeg(iCurv,:);
            if norm(v1) < 1e-3 || norm(v2) < 1e-3, continue; end
            c = dot(v1,v2)/(norm(v1)*norm(v2));
            c = max(min(c,1),-1);          % Constrain to [-1,1]
            ang = acos(c);                 % Radians
            turn_measure = turn_measure + ang;
        end

        % Longer segments and larger turning angles result in larger max_wp
        len_scale  = min(3.0, 1.0 + seg_len / 300);        % Scale by 1x per 300m, max 3x
        turn_scale = min(3.0, 1.0 + turn_measure / pi);    % Scale by 1x per π radians, max 3x

        adaptive_max_wp = round(base_max_wp * len_scale * turn_scale);
        adaptive_max_wp = max(base_max_wp, min(adaptive_max_wp, 5*base_max_wp)); % Constrain to [base, 5*base]

        % Call key point selection function with adaptive max_wp (current_pos using sp)
        waypoints = selectKeyPointsByCurvature(rawSeg, sp, adaptive_max_wp);

        % Minimum safeguard: at least 6 points (including start/end) to prevent degeneration
        min_points = min(5, adaptive_max_wp);
        if size(waypoints,1) < min_points
            % Uniformly supplement points from entire rawSeg to reach min_points
            idx_uniform = round(linspace(1, size(rawSeg,1), min_points));
            wp_extra = rawSeg(idx_uniform, :);
            waypoints = unique([waypoints; wp_extra], 'rows', 'stable');
        end

        % Ensure start/end = sp / gp
        if norm(waypoints(1,:) - sp) > 1e-3
            waypoints(1,:) = sp;
        end
        if norm(waypoints(end,:) - gp) > 1e-3
            waypoints(end,:) = gp;
        end

        fprintf('      Segment length≈%.1fm, total turn≈%.2frad, baseline max_wp=%d, adaptive max_wp=%d, actual key points=%d\n', ...
            seg_len, turn_measure, base_max_wp, adaptive_max_wp, size(waypoints,1));

        % 3) Time allocation
        t = adaptiveTimeAllocation(waypoints, [0;0;0], ...
            params.speed_limit, params.acc_max, params.jerk_max);

        % 4) MinSnap mode: All segments can be treated as "flying from A to B"
        mode = struct();
        mode.isTaskSegment  = true;
        mode.isFinalSegment = (seg == numNodes - 1);

        % Initial state (position only + zero velocity)
        if isempty(curr_state)
            curr_state = [waypoints(1,1:3)'; 0; 0; 0];
        else
            curr_state(1:3) = waypoints(1,1:3)';
            curr_state(4:6) = [0;0;0];
        end

        % 5) Call MinSnap solver
        [traj_seg, success, ~] = solveMinimumSnapForUAV( ...
            waypoints, curr_state, params, t, mode);

        if ~success || isempty(traj_seg)
            fprintf('      ✗ MinSnap failed, using BiRRT sub-path\n');
            segPath = rawSeg;
        else
            % Trajectory [N×6], take first three columns
            segPath = traj_seg(:,1:3);
            
            % Hard safeguard: height not less than 0
            if size(segPath,2) >= 3
                segPath(:,3) = max(segPath(:,3), 0);
            end

            % Safety check: avoid buildings
            try
                if ~isempty(omap)
                    occ = checkOccupancy(omap, segPath(:,1:3));
                    if any(occ > 0.8)
                        fprintf('      ⚠ MinSnap sub-path near obstacles, reverting to BiRRT sub-path\n');
                        segPath = rawSeg;
                    end
                end
            catch
                fprintf('      ⚠ Collision check exception, keeping MinSnap result\n');
            end
        end

        % 6) Concatenate to global path
        if isempty(fullPath)
            fullPath = segPath;
        else
            fullPath = [fullPath; segPath(2:end,:)];
        end
    end

    optimizedRefPaths{u} = fullPath;
    fprintf('\n  [UAV%d] Segmented MinSnap completed, total points: %d\n', u, size(fullPath,1));
end

fprintf('\n[Segmented MinSnap] All UAV segmented smoothing completed.\n');
end