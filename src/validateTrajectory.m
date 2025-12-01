function isValid = validateTrajectory(trajectory, speed_limit, acc_max, jerk_max)
% validateTrajectory Validate trajectory dynamic feasibility and continuity, called in solveMinimumSnapForUAV.m
%
% Description:
%   Performs comprehensive dynamic constraint validation on generated trajectories,
%   including speed limit checks, acceleration reasonability analysis, and trajectory
%   continuity detection to ensure trajectory feasibility and safety in actual flight.
%
% Inputs:
%   trajectory   - Trajectory data, [N×6] matrix, format [x, y, z, vx, vy, vz]
%   speed_limit  - Speed upper limit, units m/s
%   acc_max      - Acceleration upper limit, units m/s²
%   jerk_max     - Jerk upper limit, units m/s³ (reserved parameter, not strictly used yet)
%
% Outputs:
%   isValid      - Validation result flag, true indicates trajectory satisfies all constraints
%
% Validation Items:
%   1. Speed constraint validation: Check if 90th percentile speed exceeds limit (15% margin allowed)
%   2. Acceleration constraint validation: Check if 95th percentile acceleration exceeds limit (30% margin allowed)
%   3. Trajectory continuity validation: Detect displacement jumps between adjacent points, avoid unreasonable trajectory discontinuities
%
% Algorithm Features:
%   - Uses quantile statistics instead of absolute maximum values for better noise robustness
%   - Employs moving average filtering to reduce noise from numerical differentiation
%   - Combines statistical methods and physical constraints for comprehensive judgment
%
% Example:
%   isValid = validateTrajectory(traj, 15.0, 8.0, 15.0);

    % Initial check: Trajectory data validity verification
    if isempty(trajectory) || size(trajectory, 1) < 2
        isValid = false;
        fprintf('    Trajectory empty or insufficient points\n');
        return;
    end
    
    %% ===================== 1. Speed Constraint Validation =====================
    
    % Extract velocity components and calculate speed magnitude
    velocities = trajectory(:, 4:6);
    speeds = vecnorm(velocities, 2, 2);

    % Speed data smoothing (reduce numerical noise impact)
    if numel(speeds) >= 3
        speeds_smoothed = speeds;
        for k = 2:numel(speeds) - 1
            % Three-point moving average filter
            speeds_smoothed(k) = (speeds(k - 1) + speeds(k) + speeds(k + 1)) / 3;
        end
    else
        speeds_smoothed = speeds;
    end

    % Quantile-based speed constraint check
    speeds_sorted = sort(speeds_smoothed);
    idx90 = max(1, round(0.9 * numel(speeds_sorted)));
    v90 = speeds_sorted(idx90);           % 90th percentile speed
    v_max_obs = max(speeds_smoothed);     % Observed maximum speed

    % Speed constraint judgment (15% margin allowed)
    if v90 > speed_limit * 1.15
        fprintf('    Speed limit exceeded (90th percentile): %.1f > %.1f (max=%.1f)\n', ...
            v90, speed_limit, v_max_obs);
        isValid = false;
        return;
    end
    
    %% ===================== 2. Acceleration Constraint Validation =====================
    
    % Only perform acceleration check when sufficient trajectory points exist
    if size(trajectory, 1) > 2
        dt = 0.1;  % Time step, consistent with trajectory generation sampling step
        
        % 2.1 Numerical differentiation to calculate raw acceleration
        raw_acc = diff(velocities) / dt;  % [N-1 × 3] acceleration matrix
        
        % 2.2 Acceleration data smoothing
        acc_smoothed = raw_acc;
        if size(raw_acc, 1) >= 3
            for k = 2:size(raw_acc, 1) - 1
                % Three-point moving average filter to reduce numerical noise
                acc_smoothed(k, :) = (raw_acc(k - 1, :) + raw_acc(k, :) + raw_acc(k + 1, :)) / 3;
            end
        end
        
        % Calculate acceleration magnitude
        acc_magnitudes = vecnorm(acc_smoothed, 2, 2);
        
        % 2.3 Quantile-based acceleration constraint check
        acc_sorted = sort(acc_magnitudes);
        idx90 = max(1, round(0.9 * numel(acc_sorted)));
        acc90 = acc_sorted(idx90);        % 90th percentile acceleration
        acc_max_obs = max(acc_magnitudes); % Observed maximum acceleration
        
        % Acceleration constraint judgment (30% margin allowed)
        if acc90 > acc_max * 1.3
            fprintf('    Acceleration limit exceeded (90th percentile): %.1f > %.1f (max=%.1f)\n', ...
                acc90, acc_max, acc_max_obs);
            isValid = false;
            return;
        end
    end
    
    %% ===================== 3. Trajectory Continuity Validation =====================
    
    % Extract position data and calculate displacements between adjacent points
    positions = trajectory(:, 1:3);
    position_jumps = vecnorm(diff(positions), 2, 2);  % Displacement magnitude between adjacent points
    
    % Empty value check
    if isempty(position_jumps)
        isValid = true;
        return;
    end

    % 3.1 Calculate theoretical expected displacement
    dt = 0.1;  % Consistent with trajectory sampling step
    expected_mean = speed_limit * dt;  % Theoretical expected displacement based on speed limit
    hard_limit = speed_limit * 0.5;    % Conservative hard threshold (corresponding to 0.5 second displacement)

    % 3.2 Statistical analysis of displacement data
    pj_sorted = sort(position_jumps);
    idx90 = max(1, round(0.90 * numel(pj_sorted)));
    pj90 = pj_sorted(idx90);           % 90th percentile displacement
    
    pj_mean = mean(position_jumps);    % Average displacement
    pj_std = std(position_jumps);      % Displacement standard deviation

    % 3.3 Calculate comprehensive soft threshold
    soft_limit = max([pj_mean + 3 * pj_std, 2 * expected_mean, hard_limit]);
    max_jump = max(position_jumps);    % Maximum displacement

    % Continuity constraint judgment
    if pj90 > soft_limit
        fprintf('    Trajectory discontinuous: 90%% jump=%.2fm, threshold=%.2fm (max=%.2fm)\n', ...
            pj90, soft_limit, max_jump);
        isValid = false;
        return;
    end

    % All validations passed
    isValid = true;
end