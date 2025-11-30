function path_positions = birrt_plan_segment(startPt, goalPt, xMin, xMax, yMin, yMax, zMin, zMax, omap)
    % BIRRT_SEGMENT_PLANNER Bidirectional Rapidly-exploring Random Tree Path Planner (Enhanced Version)
    %
    % DESCRIPTION:
    %   Uses BiRRT algorithm to plan collision-free path from start to goal in occupancyMap3D
    %   Includes multiple safety enhancement mechanisms: loop detection & removal, interpolation, 
    %   smoothing, collision verification to prevent wall penetration, path validation & repair
    %
    % MAIN PROCESS:
    %   1) Create stateSpaceSE3 and validatorOccupancyMap3D (ValidationDistance = 2.0)
    %   2) Construct SE3 start/goal states and correct with ensureValidState (priority: Z-axis, then XY-plane)
    %   3) Configure and run plannerBiRRT (MaxConnectionDistance=30, MaxIterations=5000)
    %   4) If solution found: extract points → removeLoops → interpolate/smooth → 
    %      point-by-point collision verification & bypass repair → return path_positions
    %   5) If exception or failure: print message and return empty
    %
    % INPUT PARAMETERS:
    %   startPt - Start point coordinates [x, y, z]
    %   goalPt  - Goal point coordinates [x, y, z]  
    %   xMin, xMax, yMin, yMax, zMin, zMax - State space boundaries
    %   omap - occupancyMap3D object containing obstacle information
    %
    % OUTPUT PARAMETERS:
    %   path_positions - Planned path point sequence [N×3 matrix], returns empty if failed

    % Initialize return path
    path_positions = [];
    
    try
        % =================================================================
        % 1. State Space and Validator Configuration (Enhanced Collision Detection)
        % =================================================================
        
        % Create SE3 state space (includes position and quaternion orientation)
        ss = stateSpaceSE3([xMin xMax; yMin yMax; zMin zMax; -1 1; -1 1; -1 1; -1 1]);
        
        % Create occupancy map validator (increased validation distance for safety)
        sv = validatorOccupancyMap3D(ss, 'Map', omap, 'ValidationDistance', 2.0);
        
        % =================================================================
        % 2. Start/Goal State Definition and Validity Verification
        % =================================================================
        
        % Define start state [x, y, z, qw, qx, qy, qz] (default orientation)
        start_state = [startPt(1), startPt(2), startPt(3), 1, 0, 0, 0];
        goal_state  = [goalPt(1), goalPt(2), goalPt(3), 1, 0, 0, 0];
        
        % Verify and automatically adjust start/goal states (ensure not inside obstacles)
        start_state = ensureValidState(sv, start_state, omap);
        goal_state  = ensureValidState(sv, goal_state, omap);
        
        % Check start/goal validity
        if isempty(start_state) || isempty(goal_state)
            fprintf('    [BiRRT Warning] Start or goal point invalid, path planning terminated\n');
            return;
        end
        
        % =================================================================
        % 3. BiRRT Planner Parameter Configuration (Conservative Strategy for Safety)
        % =================================================================
        
        planner = plannerBiRRT(ss, sv);
        planner.MaxConnectionDistance = 30;    % Reduced to minimize wall penetration risk
        planner.MaxIterations = 5000;          % Increased to improve success rate
        planner.EnableConnectHeuristic = true; % Enable connection heuristic
        
        % =================================================================
        % 4. Execute Path Planning
        % =================================================================
        
        [pthObj, solnInfo] = plan(planner, start_state, goal_state);
        
        % =================================================================
        % 5. Path Post-processing and Optimization
        % =================================================================
        
        if solnInfo.IsPathFound && ~isempty(pthObj)
            % 5.1 Extract raw path points (position information only)
            rawPath = extractPathPoints(pthObj);
            
            if isempty(rawPath) || size(rawPath, 1) < 2
                return;  % Insufficient path points, return directly
            end
            
            % 5.2 Remove loops from path (avoid无效 detours)
            rawPath = removeLoops(rawPath);
            
            % 5.3 Path interpolation, smoothing and validation
            if size(rawPath, 1) >= 2
                path_positions = processAndValidatePath(rawPath, omap);
            else
                path_positions = rawPath;  % Use raw path directly
            end
        end
        
    catch ME
        % Exception handling: log error and return empty path
        fprintf('    [BiRRT Exception] Path planning process error: %s\n', ME.message);
        path_positions = [];
    end
end


%% ========================================================================
%% Helper Function 1: Path Point Extraction
%% ========================================================================

function pathPoints = extractPathPoints(pathObject)
    % EXTRACT_PATH_POINTS Extract position points from path object
    %
    % FUNCTION: Compatible with different versions of path object data structures,
    %           extracts first three coordinates
    %
    % INPUT: pathObject - Path planning result object
    % OUTPUT: pathPoints - Path point coordinate sequence [N×3]

    pathPoints = [];
    
    % Method 1: Check States field
    if isfield(pathObject, 'States')
        pathPoints = pathObject.States(:, 1:3);
    
    % Method 2: Check States property  
    elseif isprop(pathObject, 'States')
        pathPoints = pathObject.States(:, 1:3);
    end
    
    % Verify extraction result
    if isempty(pathPoints)
        fprintf('    [Path Extraction] Cannot extract valid points from path object\n');
    end
end


%% ========================================================================
%% Helper Function 2: State Validity Assurance
%% ========================================================================

function validState = ensureValidState(validator, state, omap)
    % ENSURE_VALID_STATE Ensure state is in free space
    %
    % FUNCTION: If given state is inside obstacle, automatically search for valid state nearby
    % STRATEGY: Priority adjustment in Z-axis, then XY-plane adjustment
    %
    % INPUT: validator - State validator object
    %        state     - State to validate [x,y,z,qw,qx,qy,qz]  
    %        omap      - Occupancy grid map
    % OUTPUT: validState - Adjusted valid state, returns empty if not found

    % Check if original state is valid
    if isStateValid(validator, state)
        validState = state;
        return;
    end
    
    % Strategy 1: Z-axis adjustment (priority attempt)
    for dz = [5, 10, 15, -5, -10, -15]
        testState = state;
        testState(3) = state(3) + dz;  % Adjust height
        
        if isStateValid(validator, testState)
            fprintf('    [State Adjustment] Obtained valid state via height adjustment %.0fm\n', dz);
            validState = testState;
            return;
        end
    end
    
    % Strategy 2: XY-plane adjustment (secondary attempt)
    for dx = [-10, -5, 5, 10]
        for dy = [-10, -5, 5, 10]
            testState = state;
            testState(1) = state(1) + dx;  % Adjust X coordinate
            testState(2) = state(2) + dy;  % Adjust Y coordinate
            
            if isStateValid(validator, testState)
                fprintf('    [State Adjustment] Obtained valid state via horizontal adjustment [%.0f,%.0f]m\n', dx, dy);
                validState = testState;
                return;
            end
        end
    end
    
    % All adjustment strategies failed
    fprintf('    [State Adjustment] Cannot find valid state near start/goal point\n');
    validState = [];
end


%% ========================================================================
%% Helper Function 3: Path Loop Removal
%% ========================================================================

function cleanPath = removeLoops(path)
    % REMOVE_LOOPS Detect and remove loops from path
    %
    % FUNCTION: Identify无效 detours (loops) in path and skip middle segments
    % PRINCIPLE: If subsequent point is very close to current point, loop detected
    %
    % INPUT: path - Original path point sequence [N×3]
    % OUTPUT: cleanPath - Loop-removed path [M×3], M ≤ N

    % Too few path points, no processing needed
    if size(path, 1) < 10
        cleanPath = path;
        return;
    end
    
    % Initialize clean path (start from first point)
    cleanPath = path(1, :);
    i = 1;  % Current processing point index
    
    % Traverse path points to detect loops
    while i < size(path, 1)
        bestJ = i + 1;  % Default next point
        minDist = inf;   % Minimum loop distance
        
        % Search forward for possible loop points (skip neighbors to avoid misjudgment)
        for j = i+5:min(i+30, size(path, 1))
            dist = norm(path(i, :) - path(j, :));
            
            % Loop detected: subsequent point too close to current point
            if dist < 20 && j > bestJ
                bestJ = j;
                minDist = dist;
            end
        end
        
        % Process detected loop
        if minDist < 20 && bestJ > i + 5
            % Skip loop segment, directly connect to loop end point
            cleanPath = [cleanPath; path(bestJ, :)];
            fprintf('    [Loop Removal] Loop detected: skipped %d intermediate points\n', bestJ - i - 1);
            i = bestJ;
        else
            % No loop, normally add next point
            cleanPath = [cleanPath; path(i+1, :)];
            i = i + 1;
        end
    end
    
    % Ensure goal point is included in path
    if norm(cleanPath(end, :) - path(end, :)) > 10
        cleanPath = [cleanPath; path(end, :)];
        fprintf('    [Loop Removal] Re-added goal point to ensure completeness\n');
    end
end


%% ========================================================================
%% Helper Function 4: Path Processing and Validation
%% ========================================================================

function validPath = processAndValidatePath(rawPath, omap)
    % PROCESS_AND_VALIDATE_PATH Path interpolation, smoothing and collision verification
    %
    % FUNCTION: Perform density adjustment and smoothing on raw path, and verify safety
    % PROCESS: Interpolation → Smoothing → Segment verification → Bypass repair
    %
    % INPUT: rawPath - Raw path points [N×3]
    %        omap    - Occupancy grid map
    % OUTPUT: validPath - Processed safe path [M×3]

    % ---------------------------------------------------------------------
    % Step 1: Path Interpolation (Increase path point density)
    % ---------------------------------------------------------------------
    
    % Calculate total path length
    segmentLengths = vecnorm(diff(rawPath, 1, 1), 2, 2);
    totalDist = sum(segmentLengths);
    
    % Dynamically determine interpolation points (one point every 8 meters)
    numInterpolated = max(15, ceil(totalDist / 8));
    
    % Parametric interpolation
    t_original = linspace(0, 1, size(rawPath, 1));
    t_interpolated = linspace(0, 1, numInterpolated);
    
    % Piecewise Cubic Hermite Interpolating Polynomial (shape preserving)
    interpolatedPath = [
        interp1(t_original, rawPath(:,1), t_interpolated, 'pchip')', ...
        interp1(t_original, rawPath(:,2), t_interpolated, 'pchip')', ...
        interp1(t_original, rawPath(:,3), t_interpolated, 'pchip')'
    ];
    
    % ---------------------------------------------------------------------
    % Step 2: Mild Smoothing (Avoid over-smoothing causing wall penetration)
    % ---------------------------------------------------------------------
    
    windowSize = 3;  % Moving window size (small to avoid distortion)
    smoothedPath = interpolatedPath;
    
    for dim = 1:3
        % Moving average smoothing
        smoothedPath(:, dim) = movmean(interpolatedPath(:, dim), windowSize);
    end
    
    % ---------------------------------------------------------------------
    % Step 3: Strict Path Verification and Repair
    % ---------------------------------------------------------------------
    
    validPath = validateAndFixPath(smoothedPath, omap);
    
    % Backup strategy: If smoothed path verification fails, try original interpolated path
    if isempty(validPath) || size(validPath, 1) < 2
        fprintf('    [Path Processing] Smoothed path verification failed, trying original interpolated path\n');
        validPath = validateAndFixPath(interpolatedPath, omap);
    end
    
    % Final backup: Use raw path directly
    if isempty(validPath)
        fprintf('    [Path Processing] All processing failed, using raw path\n');
        validPath = rawPath;
    end
end


%% ========================================================================
%% Helper Function 5: Path Verification and Repair
%% ========================================================================

function validPath = validateAndFixPath(path, omap)
    % VALIDATE_AND_FIX_PATH Point-by-point path verification and collision segment repair
    %
    % FUNCTION: Check safety of each path point and segment, attempt bypass when collision detected
    % STRATEGY: Point collision check + Segment collision check + Intelligent bypass
    %
    % INPUT: path - Path to verify [N×3]  
    %        omap - Occupancy grid map
    % OUTPUT: validPath - Verified and repaired safe path [M×3]

    validPath = [];  % Initialize valid path
    
    % Traverse all path points
    for i = 1:size(path, 1)
        % Check if current point is in free space (strict threshold)
        occVal = checkOccupancy(omap, path(i, :));
        
        if occVal < 0.5  % Free space threshold
            if isempty(validPath)
                % First valid point, add directly
                validPath = path(i, :);
            else
                % Check line segment safety with previous valid point
                lastPoint = validPath(end, :);
                
                if ~checkLineCollision(lastPoint, path(i, :), omap)
                    % Segment safe, add current point
                    validPath = [validPath; path(i, :)];
                else
                    % Segment penetrates wall, attempt to find bypass point
                    fprintf('    [Path Repair] Line segment wall penetration detected, searching for bypass point...\n');
                    bypassPoint = findBypassPoint(lastPoint, path(i, :), omap);
                    
                    if ~isempty(bypassPoint)
                        % Successfully found bypass point
                        validPath = [validPath; bypassPoint; path(i, :)];
                        fprintf('    [Path Repair] Successfully inserted bypass point\n');
                    else
                        fprintf('    [Path Repair] Cannot find suitable bypass point, skipping current segment\n');
                    end
                end
            end
        else
            fprintf('    [Path Verification] Path point %d inside obstacle, skipping\n', i);
        end
    end
    
    % Verify final path validity
    if size(validPath, 1) < 2
        fprintf('    [Path Verification] Insufficient valid path points, returning empty path\n');
        validPath = [];
    end
end


%% ========================================================================
%% Helper Function 6: Line Segment Collision Detection
%% ========================================================================

function hasCollision = checkLineCollision(p1, p2, omap)
    % CHECK_LINE_COLLISION Detect if line between two points penetrates obstacles
    %
    % FUNCTION: Sample multiple points along line segment for collision detection
    % PRINCIPLE: Uniform sampling + point-by-point occupancy value check
    %
    % INPUT: p1, p2 - Line segment endpoint coordinates [x,y,z]
    %        omap   - Occupancy grid map
    % OUTPUT: hasCollision - Boolean, true indicates collision

    % Calculate segment length and sampling points (one sample every 2 meters)
    segmentLength = norm(p2 - p1);
    numChecks = max(2, ceil(segmentLength / 2));  % Minimum 2 sample points
    
    hasCollision = false;
    
    % Uniform sampling detection along line segment
    for t = linspace(0, 1, numChecks)
        % Calculate sample point coordinates
        point = p1 + t * (p2 - p1);
        
        % Check sample point occupancy status
        occVal = checkOccupancy(omap, point);
        
        if occVal >= 0.5  % Collision threshold
            hasCollision = true;
            fprintf('    [Collision Detection] Collision found at %.1f%% of line segment\n', t*100);
            return;  % Return immediately upon collision detection
        end
    end
end


%% ========================================================================
%% Helper Function 7: Bypass Point Search
%% ========================================================================

function bypassPoint = findBypassPoint(p1, p2, omap)
    % FIND_BYPASS_POINT Search for feasible bypass point around obstacles
    %
    % FUNCTION: When straight path is blocked, search for feasible bypass point around midpoint
    % STRATEGY: Horizontal bypass → Vertical bypass
    %
    % INPUT: p1, p2 - Two endpoints of blocked line segment
    %        omap   - Occupancy grid map  
    % OUTPUT: bypassPoint - Found bypass point coordinates, returns empty if not found

    % Calculate segment midpoint
    midPoint = (p1 + p2) / 2;
    
    % Calculate segment direction vector
    direction = p2 - p1;
    
    % Calculate perpendicular direction (in XY plane)
    if norm(direction(1:2)) > 0.1
        perpendicular = [-direction(2), direction(1), 0];
        perpendicular = perpendicular / norm(perpendicular) * 10;  % 10 meter offset
    else
        % Handle approximately vertical segments
        perpendicular = [10, 0, 0];
    end
    
    % ---------------------------------------------------------------------
    % Strategy 1: Horizontal Bypass (try left and right sides)
    % ---------------------------------------------------------------------
    
    for offset = [perpendicular; -perpendicular]'
        testPoint = midPoint + offset';
        testPoint(3) = (p1(3) + p2(3)) / 2;  % Maintain original height
        
        % Check bypass point itself and safety of two connecting segments
        occVal = checkOccupancy(omap, testPoint);
        if occVal < 0.5
            segment1Safe = ~checkLineCollision(p1, testPoint, omap);
            segment2Safe = ~checkLineCollision(testPoint, p2, omap);
            
            if segment1Safe && segment2Safe
                fprintf('    [Bypass Search] Found horizontal bypass point\n');
                bypassPoint = testPoint;
                return;
            end
        end
    end
    
    % ---------------------------------------------------------------------
    % Strategy 2: Vertical Bypass (lift upward)
    % ---------------------------------------------------------------------
    
    testPoint = midPoint + [0, 0, 15];  % Lift upward 15 meters
    occVal = checkOccupancy(omap, testPoint);
    
    if occVal < 0.5
        segment1Safe = ~checkLineCollision(p1, testPoint, omap);
        segment2Safe = ~checkLineCollision(testPoint, p2, omap);
        
        if segment1Safe && segment2Safe
            fprintf('    [Bypass Search] Found vertical bypass point\n');
            bypassPoint = testPoint;
            return;
        end
    end
    
    % All bypass strategies failed
    fprintf('    [Bypass Search] Cannot find suitable bypass point\n');
    bypassPoint = [];
end