function isFeasible = checkKinematicFeasibility(trajectories, vMax, aMax)
% checkKinematicFeasibility Check kinematic feasibility of multiple trajectories
%
% Description:
%   Evaluates whether each trajectory in a set satisfies kinematic constraints
%   including maximum velocity and acceleration limits. Uses statistical
%   analysis for acceleration checking to handle transient peaks while
%   ensuring overall feasibility.
%
% Input Parameters:
%   trajectories - Cell array of trajectories, each [N×6] with columns:
%                 [x, y, z, vx, vy, vz] representing position and velocity
%   vMax        - Maximum allowed velocity (m/s)
%   aMax        - Maximum allowed acceleration (m/s²)
%
% Output Parameters:
%   isFeasible  - Logical vector indicating feasibility of each trajectory
%                 true: trajectory satisfies kinematic constraints
%                 false: trajectory violates constraints
%
% Algorithm Features:
%   - Velocity check: Allows 10% tolerance above vMax for robustness
%   - Acceleration check: Uses 90th percentile value with 30% tolerance
%   - Handles edge cases: empty trajectories, insufficient data points
%   - Assumes fixed time step of 0.1s (consistent with planning/MinSnap)
%
% Examples:
%   % Check feasibility of multiple trajectories
%   feasible = checkKinematicFeasibility(trajSet, 20.0, 8.0);

    isFeasible = false(length(trajectories), 1);
    
    for i = 1:length(trajectories)
        traj = trajectories{i};
        if isempty(traj) || size(traj,2) < 6
            isFeasible(i) = false;
            continue;
        end
        
        velocities = traj(:, 4:6);
        speeds     = vecnorm(velocities, 2, 2);
        
        dt = 0.1;   % Consistent with planning/MinSnap sampling
        
        % Calculate acceleration
        if size(velocities, 1) > 2
            accelerations = diff(velocities, 1, 1) / dt;   % [K-1 × 3]
            accMag        = vecnorm(accelerations, 2, 2);  % [K-1 × 1]
        else
            accMag = [];
        end
        
        % Velocity check (slightly relaxed: allow up to 1.1 * vMax)
        speedOK = all(speeds >= -1e-3 & speeds <= vMax * 1.10);
        
        % Acceleration check: use 90th percentile + 30% margin
        if isempty(accMag)
            accOK = true;
        else
            acc_sorted = sort(accMag);
            idx90      = max(1, round(0.90 * numel(acc_sorted)));
            acc90      = acc_sorted(idx90);
            
            accOK = acc90 <= aMax * 1.30;
        end
        
        isFeasible(i) = speedOK && accOK;
    end
end