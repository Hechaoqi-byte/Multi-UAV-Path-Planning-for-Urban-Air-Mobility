function planner = helperCreateTrajectoryPlanner(refPath, egoUAV)
% helperCreateTrajectoryPlanner Create and initialize trajectory planner structure
%
% Description:
%   Initializes a trajectory planner structure with configuration parameters
%   for motion planning and trajectory generation. The planner is designed
%   to work with Model Predictive Control (MPC) approaches and generates
%   multiple candidate trajectories with different speed profiles and 
%   lateral offsets for obstacle avoidance.
%
% Input Parameters:
%   refPath - Reference path structure containing:
%             .xyz      - Path points [N×3]
%             .s        - Cumulative arc length [N×1]
%             .tangents - Tangent vectors [N×3]
%             .sMax     - Total path length
%   egoUAV  - Ego UAV state information (currently unused but reserved for future extensions)
%
% Output Parameters:
%   planner - Trajectory planner structure with fields:
%             .refPath          - Reference path structure
%             .timeHorizon      - Prediction time horizon (seconds)
%             .dt               - Time step (seconds)
%             .numSpeeds        - Number of speed samples for trajectory generation
%             .numLateralOffsets- Number of lateral offset samples
%             .lateralRange     - Lateral maneuver range [min, max] in meters
%             .speedConfig      - Speed configuration substructure
%
% Configuration Details:
%   - Time Domain: 3.0s horizon with 0.1s steps (consistent with MinSnap sampling)
%   - Sampling: 6 speed profiles × 3 lateral offsets = 18 candidate trajectories
%   - Lateral Maneuvers: ±3 meters maximum lateral displacement
%   - Speed Settings: Configurable min/max/preferred speeds

    % Create trajectory planner - speed options consistent with global configuration
    
    planner = struct();
    planner.refPath = refPath;
    
    % Time domain parameters: consistent with main loop
    planner.timeHorizon = 3.0;   % 3-second prediction
    planner.dt = 0.1;            % 0.1-second step (consistent with MinSnap sampling)
    
    % Sampling parameters
    planner.numSpeeds = 6;              % Number of speed samples
    planner.numLateralOffsets = 3;      % Number of lateral offset samples
    planner.lateralRange = [-3 3];      % Maximum left/right 3m maneuver
    
    % Speed configuration (only default values here, actual vMax controlled by external speedLimit)
    planner.speedConfig = struct();
    planner.speedConfig.minSpeed       = 0.1;    % Allow stopping
    planner.speedConfig.maxSpeed       = 20;   % Default upper limit (controlled externally by CONFIG.speed_limit)
    planner.speedConfig.preferredSpeed = 12;   % Preferred speed, slightly less than 15 global constraint
end