function t = adaptiveTimeAllocation(waypoints, current_vel, speed_limit, acc_max, jerk_max)
    % Adaptive time allocation algorithm for trajectory planning
    % Input parameters:
    %   waypoints    - Waypoint sequence [n x 3]
    %   current_vel  - Current velocity vector [1 x 3]  
    %   speed_limit  - Speed limit
    %   acc_max      - Maximum acceleration
    %   jerk_max     - Maximum jerk
    % Output parameters:
    %   t - Time point sequence [1 x n]
    
    % Parameter checking and default values
    if nargin < 5
        jerk_max = 10.0;  % Default maximum jerk
    end
    
    % Input validation
    n = size(waypoints, 1);
    if n < 2
        t = [0, 1.0];  % Minimum 2 points required, return default time
        return;
    end
    
    % Calculate path segment information
    segment_lengths = zeros(1, n-1);
    segment_directions = zeros(n-1, 3);
    
    for i = 1:n-1
        % Calculate path vector
        vec = waypoints(i+1, :) - waypoints(i, :);
        segment_lengths(i) = norm(vec);
        
        % Calculate unit direction vector
        if segment_lengths(i) > 0
            segment_directions(i, :) = vec / segment_lengths(i);
        else
            segment_directions(i, :) = [0, 0, 0];  % Zero-length segment
        end
    end
    
    % Time allocation calculation
    t = zeros(1, n);
    t(1) = 0;  % Start time is 0
    
    for i = 1:n-1
        dist = segment_lengths(i);
        
        % Handle zero-length segments
        if dist < 1e-6
            segment_time = 0.1;  % Give minimum time for zero-length segments
        else
            % Time calculation based on jerk constraints
            % Calculate jerk-limited phase time
            T_jerk = min(sqrt(speed_limit / jerk_max), acc_max / jerk_max);
            % Calculate acceleration-limited phase time  
            T_acc = speed_limit / acc_max - T_jerk;
            % Calculate constant velocity phase time
            T_const = dist / speed_limit - T_acc - 2 * T_jerk;
            
            % Check if constant velocity phase exists
            if T_const > 0
                % Three-phase: jerk-acceleration + constant-acceleration + constant-velocity + jerk-deceleration + constant-deceleration
                segment_time = 2 * T_jerk + T_acc + T_const;
            else
                % Two-phase: cannot reach maximum velocity, only jerk-acceleration and jerk-deceleration
                v_max = sqrt(acc_max * dist / (1 + acc_max / jerk_max));
                T_jerk_new = v_max / acc_max;
                T_acc_new = v_max / acc_max - T_jerk_new;
                segment_time = 2 * T_jerk_new + T_acc_new;
            end
            
            % Consider the influence of initial velocity
            if i == 1 && norm(current_vel) > 0
                current_speed = norm(current_vel);
                current_dir = current_vel / current_speed;
                dot_product = dot(current_dir, segment_directions(i, :));
                
                % Velocity direction aligns with path direction: can reduce time
                if dot_product > 0.9
                    decel_time = current_speed / acc_max;
                    segment_time = max(segment_time - decel_time * 0.5, segment_time * 0.7);
                    
                % Velocity direction opposes path direction: need to increase time
                elseif dot_product < -0.9
                    brake_time = current_speed / acc_max;
                    segment_time = segment_time + brake_time;
                end
            end
            
            % Apply minimum time from various constraints
            min_time_by_speed = dist / speed_limit;      % Speed constraint
            min_time_by_acc = sqrt(2 * dist / acc_max);  % Acceleration constraint
            segment_time = max([segment_time, min_time_by_speed, min_time_by_acc, 0.5]);
            
            % Add safety margin to avoid generating too aggressive trajectories
            safety_scale = 3;  % Safety factor, can be adjusted based on actual debugging
            segment_time = segment_time * safety_scale;
        end
        
        % Accumulate time
        t(i+1) = t(i) + segment_time;
    end
    
    % Output debug information
    fprintf('  Time allocation: Total time %.1fs, %d segments\n', t(end), n-1);
end