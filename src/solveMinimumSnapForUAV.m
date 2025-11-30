function [trajectory, success, solution_info] = solveMinimumSnapForUAV( ...
    waypoints, current_state, params, time_allocation, mode)
% solveMinimumSnapForUAV UAV trajectory optimizer based on Minimum Snap principle
%
% Description:
%   Generates smooth UAV trajectories using Minimum Snap (minimizing 4th derivative)
%   method while satisfying dynamic constraints and ensuring trajectory continuity
%   and feasibility. This function is the core optimization module for UAV trajectory generation.
%
% Inputs:
%   waypoints       - Waypoint sequence, [N×3] matrix, each row represents a 3D waypoint
%   current_state   - Current state vector, [9×1] or [6×1], containing position, velocity, acceleration
%   params          - Optimization parameter structure, containing polynomial order, weight coefficients, etc.
%   time_allocation - Time allocation vector, specifying time duration for each trajectory segment
%   mode            - Trajectory mode structure, defining special constraints for task segments and final segments
%
% Outputs:
%   trajectory      - Optimized trajectory, [M×6] matrix containing position and velocity information
%   success         - Optimization success flag, true indicates feasible trajectory generated
%   solution_info   - Solution information structure containing detailed statistics of optimization process
%
% Algorithm Flow:
%   1. Parameter validation and default value setting
%   2. Time allocation and problem scale determination
%   3. Construct objective function matrix (minimizing Snap)
%   4. Construct equality and inequality constraint matrices
%   5. Call quadratic programming solver for optimization
%   6. Trajectory extraction and dynamic validation
%
% Example:
%   [traj, success, info] = solveMinimumSnapForUAV(wps, state, params, t, mode);

    % Initialize output parameters
    success = false;
    trajectory = [];
    solution_info = struct();

    try
        %% ===================== Parameter Validation & Default Values =====================
        
        % Handle optional input parameters
        if nargin < 5 || isempty(mode)
            mode = struct();
        end
        if ~isfield(mode, 'isTaskSegment')
            mode.isTaskSegment = true;   % Default task segment mode
        end
        if ~isfield(mode, 'isFinalSegment')
            mode.isFinalSegment = false; % Default non-final segment
        end

        % Data dimension normalization
        if size(waypoints, 2) ~= 3
            waypoints = waypoints.';     % Transpose to [N×3] format
        end
        if size(current_state, 2) > size(current_state, 1)
            current_state = current_state.'; % Convert to column vector
        end
        
        % Waypoint quantity validation
        if size(waypoints, 1) < 2
            fprintf('[Minimum Snap] Error: Insufficient waypoints\n');
            return;
        end

        %% ===================== Default Parameter Configuration =====================
        
        % Polynomial trajectory parameters
        if ~isfield(params, 'order')
            params.order = 7;            % Default 7th order polynomial
        end
        if ~isfield(params, 'k_r')
            params.k_r = 2;              % Default position+velocity+acceleration continuity
        end
        if ~isfield(params, 'k_psi')
            params.k_psi = 0;            % Default no yaw angle constraints
        end
        
        % Weight coefficient parameters
        if ~isfield(params, 'mu_r')
            params.mu_r = 1.0;           % Position weight coefficient
        end
        if ~isfield(params, 'mu_psi')
            params.mu_psi = 0.0;         % Yaw angle weight coefficient
        end
        
        % Dynamic constraint parameters
        if ~isfield(params, 'speed_limit')
            params.speed_limit = 20.0;   % Default speed limit 20 m/s
        end
        if ~isfield(params, 'acc_max')
            params.acc_max = 8.0;        % Default acceleration limit 8 m/s²
        end
        if ~isfield(params, 'jerk_max')
            params.jerk_max = 15.0;      % Default jerk limit 15 m/s³
        end
        
        % Corridor constraint parameters
        if ~isfield(params, 'corridor_enabled')
            params.corridor_enabled = true; % Default enable corridor constraints
        end
        if ~isfield(params, 'corridor_width')
            params.corridor_width = 8.0; % Default corridor width 8 meters
        end
        if ~isfield(params, 'n_intermediate')
            params.n_intermediate = 1;   % Default number of intermediate constraint points
        end

        %% ===================== Problem Scale Determination =====================
        
        m = size(waypoints, 1) - 1;      % Number of trajectory segments
        order = params.order;            % Polynomial order
        k_r = params.k_r;                % Position constraint order
        k_psi = params.k_psi;            % Yaw angle constraint order

        %% ===================== Time Allocation Processing =====================
        
        if nargin < 4 || isempty(time_allocation)
            % Adaptive time allocation
            t = adaptiveTimeAllocation(waypoints, current_state(4:6), ...
                params.speed_limit, params.acc_max, params.jerk_max);
        else
            t = time_allocation;         % Use preset time allocation
        end

        fprintf('[Minimum Snap] Segments: %d, Total time: %.1fs\n', m, t(end));

        %% ===================== Build Keyframe Matrix =====================
        
        % Keyframe format: [x-coordinate; y-coordinate; z-coordinate; yaw angle]
        keyframe = [waypoints.'; zeros(1, size(waypoints, 1))];

        fprintf('[Minimum Snap] Building QP problem...\n');

        %% ===================== Build Optimization Problem =====================
        
        % Build objective function matrix (minimizing Snap)
        A = computeA(order, m, params.mu_r, params.mu_psi, k_r, k_psi, t);

        % Build derivative constraint data
        [constraintData_r, constraintData_psi] = buildLocalConstraintData( ...
            m, k_r, k_psi, mode);

        % Build equality and inequality constraint matrices
        if params.corridor_enabled
            corridor_position = [1, m];  % Corridor constraint positions
            [C, b, Cin, bin] = computeConstraint(order, m, k_r, k_psi, t, ...
                keyframe, corridor_position, params.n_intermediate, ...
                params.corridor_width, constraintData_r, constraintData_psi);
        else
            [C, b, Cin, bin] = computeConstraint(order, m, k_r, k_psi, t, ...
                keyframe, [], [], [], constraintData_r, constraintData_psi);
        end

        % Constraint validity verification
        if isempty(C) || size(C, 1) == 0
            fprintf('[Minimum Snap] Error: No valid equality constraints\n');
            return;
        end

        %% ===================== Problem Scale Statistics =====================
        
        num_vars = size(A, 1);                   % Number of optimization variables
        num_eq_constraints = size(C, 1);         % Number of equality constraints
        num_ineq_constraints = size(Cin, 1);     % Number of inequality constraints

        fprintf('[Minimum Snap] Variables: %d, Equality constraints: %d, Inequality constraints: %d\n', ...
            num_vars, num_eq_constraints, num_ineq_constraints);

        % Matrix condition number analysis
        cond_A = cond(A);
        cond_C = cond(C);
        fprintf('[Minimum Snap] Condition numbers: A=%.2e, C=%.2e\n', cond_A, cond_C);

        %% ===================== Quadratic Programming Solution =====================
        
        % Configure optimization options
        options = optimoptions('quadprog', ...
            'Display', 'none', ...
            'MaxIterations', 4000, ...
            'ConstraintTolerance', 1e-6, ...
            'OptimalityTolerance', 1e-6);

        tic;
        try
            % Select solution method based on constraint situation
            if ~isempty(Cin) && size(Cin, 1) > 0
                % Quadratic programming with inequality constraints
                [solution, fval, exitflag] = quadprog(2*A, [], Cin, bin, C, b, ...
                    [], [], [], options);
            else
                % Quadratic programming with equality constraints only
                [solution, fval, exitflag] = quadprog(2*A, [], [], [], C, b, ...
                    [], [], [], options);
            end
            solve_time = toc;

            % Solution result verification
            if exitflag <= 0 || isempty(solution)
                fprintf('[Minimum Snap] QP solution failed (exitflag=%d)\n', exitflag);
                if exitflag == -2
                    fprintf('  Problem infeasible, possible reasons: constraint conflicts/unreasonable time allocation, etc.\n');
                end
                return;
            end

            fprintf('[Minimum Snap] QP solution successful: Objective value=%.6e, Time=%.2fs\n', ...
                fval, solve_time);

        catch QP_ME
            fprintf('[Minimum Snap] QP solution exception: %s\n', QP_ME.message);
            return;
        end

        %% ===================== Trajectory Extraction & Processing =====================
        
        % Extract trajectory from solution vector
        trajectory = extractTrajectoryFromSolution_new(solution, order, m, t);
        if isempty(trajectory)
            fprintf('[Minimum Snap] Trajectory extraction failed\n');
            return;
        end

        %% ===================== Trajectory Validation =====================
        
        % Dynamic constraint validation
        if validateTrajectory(trajectory, params.speed_limit, params.acc_max, params.jerk_max)
            success = true;
            fprintf('[Minimum Snap] ✓ Trajectory generation successful: %d points, Length=%.1fm\n', ...
                size(trajectory, 1), calculateTrajectoryLength(trajectory));
        else
            fprintf('[Minimum Snap] ✗ Trajectory validation failed, discarding this MinSnap segment\n');
            success = false;    % Mark optimization as failed
            trajectory = [];    % Clear invalid trajectory
        end

        %% ===================== Solution Information Recording =====================
        
        solution_info.solve_time = solve_time;           % Solution time
        solution_info.num_segments = m;                  % Number of trajectory segments
        solution_info.num_variables = length(solution);  % Number of variables
        solution_info.num_constraints = size(C, 1) + size(Cin, 1); % Total constraints
        solution_info.exitflag = exitflag;               % Solver exit flag
        solution_info.objective_value = fval;            % Objective function value
        solution_info.condition_number_A = cond_A;       % Objective matrix condition number
        solution_info.condition_number_C = cond_C;       % Constraint matrix condition number

    catch ME
        %% ===================== Exception Handling =====================
        
        fprintf('[Minimum Snap] Top-level error: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('  Error location: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
        success = false;
        trajectory = [];
    end
end

%% Helper Functions
function [constraintData_r, constraintData_psi] = buildLocalConstraintData( ...
    m, k_r, k_psi, mode)
% buildLocalConstraintData Build local constraint data for Minimum Snap optimization
%
% Description:
%   Generates derivative constraint data for position and yaw angle based on trajectory
%   segment characteristics and flight mode, defining boundary conditions such as velocity
%   and acceleration at key points (start, intermediate, end points) to ensure trajectory
%   physical feasibility and task adaptability.
%
% Inputs:
%   m        - Number of trajectory segments (number of waypoints - 1)
%   k_r      - Position derivative constraint order (1=velocity, 2=acceleration, ...)
%   k_psi    - Yaw angle derivative constraint order
%   mode     - Trajectory mode structure controlling endpoint and terminal behavior
%              .isTaskSegment  : Whether current trajectory endpoint is a task point (true/false)
%              .isFinalSegment : Whether current endpoint is the final endpoint of entire mission (true/false)
%
% Outputs:
%   constraintData_r   - Position constraint data, [m×k_r×3] array
%   constraintData_psi - Yaw angle constraint data, [m×k_psi×1] array
%
% Constraint Convention:
%   i = 1     : Corresponds to trajectory start point
%   i = 2..m  : Corresponds to trajectory intermediate connection points and end point
%   eps value : Indicates continuity constraint, requiring equal derivatives between segments
%   0 value   : Indicates absolute constraint, requiring zero derivative
%
% Design Logic:
%   1. Start point: Zero velocity takeoff, ensuring safe takeoff and landing
%   2. Intermediate points: Derivative continuity, ensuring trajectory smoothness
%   3. Task points: Zero velocity hover, ensuring task execution quality
%   4. Final points: Zero acceleration, achieving stable landing
%
% Example:
%   [data_r, data_psi] = buildLocalConstraintData(3, 2, 1, modeStruct);

    % Parameter default value handling
    if nargin < 4 || isempty(mode)
        mode = struct();
    end
    if ~isfield(mode, 'isTaskSegment')
        mode.isTaskSegment = true;   % Default task segment mode
    end
    if ~isfield(mode, 'isFinalSegment')
        mode.isFinalSegment = false; % Default non-final segment
    end

    %% ===================== Position Constraint Data Construction =====================
    
    % Constraint data structure: constraintData_r(i, h, k)
    % i : 1..m    Connection point index (at segment start)
    % h : 1..k_r  Derivative order (1=velocity, 2=acceleration, ...)
    % k : 1..3    Coordinate dimension (x, y, z)
    
    constraintData_r = zeros(m, k_r, 3);

    % ========== 1. Start Point Constraint Processing ==========
    
    % Start point i=1: Default stationary takeoff, ensuring safe takeoff and landing
    if k_r >= 1
        % Velocity constraint: Zero initial velocity
        constraintData_r(1, 1, 1:3) = 0;
    end
    if k_r >= 2
        % Acceleration constraint: Zero Z-axis initial acceleration
        constraintData_r(1, 2, 3) = 0;
        % Note: Can be extended to three-axis zero acceleration constraint as needed
    end

    % ========== 2. Intermediate Connection Points & End Point Constraint Processing ==========
    
    % 2.1 Intermediate connection points (i = 2..m-1): Continuity constraints
    for i = 2:max(1, m - 1)
        if k_r >= 1
            % Velocity continuity: Require equal velocity between segments
            constraintData_r(i, 1, 1:3) = eps;
        end
        if k_r >= 2
            % Acceleration continuity: Require equal acceleration between segments
            constraintData_r(i, 2, 1:3) = eps;
        end
    end

    % 2.2 End point constraint (i = m): Differentiated processing based on task mode
    if m >= 1
        % Velocity constraint processing
        if k_r >= 1
            if mode.isTaskSegment
                % Task point endpoint: Zero velocity hover, ensuring task execution accuracy
                constraintData_r(m, 1, 1:3) = 0;
            else
                % Non-task point endpoint: Velocity continuity constraint
                constraintData_r(m, 1, 1:3) = eps;
            end
        end
        
        % Acceleration constraint processing
        if k_r >= 2
            if mode.isFinalSegment
                % Final segment endpoint: Zero acceleration, achieving smooth landing
                constraintData_r(m, 2, 1:3) = 0;
            else
                % Non-final endpoint: Acceleration continuity constraint
                constraintData_r(m, 2, 1:3) = eps;
            end
        end
    end

    %% ===================== Yaw Angle Constraint Data Construction =====================
    
    constraintData_psi = zeros(m, k_psi, 1);
    
    % Yaw angle constraint processing (relatively simplified)
    if k_psi >= 1
        % Start point yaw rate constraint: Zero initial angular velocity
        constraintData_psi(1, 1, 1) = 0;
    end
    % Note: Yaw angle constraints for other points can be further extended as needed
end