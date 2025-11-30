function filter = initLidarFilter(detection)
% initLidarFilter Initialize Kalman filter for LiDAR tracking
%
% Description:
%   Initializes a 3D constant velocity Kalman filter for tracking objects
%   detected by LiDAR sensors. The filter uses position measurements from
%   LiDAR detections and initializes velocity states with zero while 
%   assigning appropriate covariance values for robust tracking performance.
%
% Input Parameters:
%   detection - LiDAR detection object containing:
%               .Measurement      - Position measurement [x, y, z] in meters
%               .MeasurementNoise - Measurement noise covariance matrix [3×3]
%
% Output Parameters:
%   filter - Initialized trackingKF object with:
%            .State           - Initial state vector [x; vx; y; vy; z; vz]
%            .StateCovariance - Initial state covariance matrix [6×6]
%            .MotionModel     - '3D Constant Velocity'
%            .ProcessNoise    - Process noise matrix
%
% State Vector:
%   [x; vx; y; vy; z; vz] where:
%     x, y, z  - Position coordinates (meters)
%     vx, vy, vz - Velocity components (m/s)
%
% Filter Configuration:
%   - Motion Model: 3D Constant Velocity assumption
%   - Initial Velocity: Zero with high uncertainty (50 m²/s² variance)
%   - Process Noise: Diagonal matrix with [0.5, 0.5, 0.5] for position noise
%   - Measurement Integration: Uses LiDAR measurement noise directly

    % Initialize LiDAR filter
    
    % Extract measurements
    meas = detection.Measurement;
    measCov = detection.MeasurementNoise;
    
    % Initial state [x; vx; y; vy; z; vz]
    state = [meas(1); 0; meas(2); 0; meas(3); 0];
    
    % Initial covariance
    stateCov = blkdiag(...
        measCov(1,1), 50, ...
        measCov(2,2), 50, ...
        measCov(3,3), 50);
    
    % Create Kalman filter
    filter = trackingKF('State', state, 'StateCovariance', stateCov, ...
        'MotionModel', '3D Constant Velocity', ...
        'ProcessNoise', diag([0.5 0.5 0.5]));
end