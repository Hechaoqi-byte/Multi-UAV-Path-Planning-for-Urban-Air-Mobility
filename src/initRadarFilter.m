function phd = initRadarFilter(detectionPartition)
% initRadarFilter Initialize GGIW-PHD filter for radar tracking
%
% Description:
%   Initializes a Gamma Gaussian Inverse Wishart Probability Hypothesis Density
%   (GGIW-PHD) filter for radar-based multi-target tracking. This filter is
%   capable of tracking extended targets with unknown extents and handling
%   measurement uncertainties. Supports both default initialization and
%   detection-based initialization modes.
%
% Input Parameters:
%   detectionPartition - (Optional) Cell array of detection partitions containing:
%                        Each detection should have:
%                        .Measurement      - Position measurement [x, y, z]
%                        .MeasurementNoise - Measurement noise covariance [3×3]
%                        If not provided, creates default empty filter
%
% Output Parameters:
%   phd - Configured ggiwphd filter object with properties:
%         .States               - State vectors [6×N] for N components
%         .StateCovariances     - State covariance matrices [6×6×N]
%         .ScaleMatrices        - Inverse Wishart scale matrices [3×3×N]
%         .DegreesOfFreedom     - Inverse Wishart degrees of freedom
%         .Shapes               - Gamma distribution shape parameters
%         .Rates                - Gamma distribution rate parameters
%         .ProcessNoise         - Process noise covariance [6×6]
%         .MeasurementFcn       - Measurement function handle (@cvmeas)
%         .StateTransitionFcn   - State transition function handle (@constvel)
%
% Filter Modes:
%   1. Default Mode (no input): Creates empty filter with default parameters
%   2. Detection Mode (with input): Initializes filter from radar detections
%
% State Vector:
%   [x; vx; y; vy; z; vz] - 6D constant velocity state
%
% Algorithm Features:
%   - Extended target tracking with unknown extent
%   - Gamma distribution for target intensity
%   - Gaussian distribution for kinematic states
%   - Inverse Wishart distribution for extent estimation
%   - Constant velocity motion model with additive noise

    % Initialize radar PHD filter (GGIW-PHD)
    % Based on official examples
    
    if nargin == 0
        % No parameters - create default filter
        
        % Process noise
        sigP = 0.2;  % Position noise
        sigV = 1;    % Velocity noise
        Q = diag([sigP, sigV, sigP, sigV, sigP, sigV].^2);
        
        phd = ggiwphd(...
            zeros(6, 0), ...              % Initial states (empty)
            repmat(eye(6), [1 1 0]), ...  % Initial covariances (empty)
            'ScaleMatrices', zeros(3, 3, 0), ...
            'MaxNumComponents', 1000, ...
            'ProcessNoise', Q, ...
            'HasAdditiveProcessNoise', true, ...
            'MeasurementFcn', @cvmeas, ...
            'MeasurementJacobianFcn', @cvmeasjac, ...
            'PositionIndex', [1 3 5], ...
            'ExtentRotationFcn', @(x, dT)eye(3, class(x)), ...
            'HasAdditiveMeasurementNoise', true, ...
            'StateTransitionFcn', @constvel, ...
            'StateTransitionJacobianFcn', @constveljac);
        
    else
        % With parameters - initialize from detections
        
        meanDetection = detectionPartition{1};
        n = numel(detectionPartition);
        
        % Collect all measurements and measurement noise
        allDets = [detectionPartition{:}];
        zAll = horzcat(allDets.Measurement);
        RAll = cat(3, allDets.MeasurementNoise);
        
        % Average measurement and noise
        z = mean(zAll, 2);
        R = mean(RAll, 3);
        meanDetection.Measurement = z;
        meanDetection.MeasurementNoise = R;
        
        % Parse detection to get position and velocity covariance
        [posMeas, velMeas, posCov] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(...
            meanDetection, 'initRadarFilter', 'double');
        
        % Create constant velocity state and covariance
        states = zeros(6, 1);
        covariances = zeros(6, 6);
        states(1:2:end) = posMeas;
        states(2:2:end) = velMeas;
        covariances(1:2:end, 1:2:end) = posCov;
        covariances(2:2:end, 2:2:end) = 10 * eye(3);
        
        % Process noise
        sigP = 0.2;
        sigV = 1;
        Q = diag([sigP, sigV, sigP, sigV, sigP, sigV].^2);
        
        %% Configure Inverse Wishart mixture parameters
        e = zAll - z;
        Z = e * e' / n + R;
        dof = 150;
        
        % Measurement Jacobian
        p = detectionPartition{1}.MeasurementParameters;
        H = cvmeasjac(states, p);
        
        Bk = H(:, 1:2:end);
        Bk2 = eye(3) / Bk;
        V = (dof - 4) * Bk2 * Z * Bk2';
        
        % Configure Gamma mixture parameters
        alpha = 16;  % Shape parameter
        beta = 16 / n;  % Rate parameter
        
        phd = ggiwphd(...
            states, covariances, ...
            'HasAdditiveMeasurementNoise', true, ...
            'ProcessNoise', Q, ...
            'HasAdditiveProcessNoise', true, ...
            'MeasurementFcn', @cvmeas, ...
            'MeasurementJacobianFcn', @cvmeasjac, ...
            'StateTransitionFcn', @constvel, ...
            'StateTransitionJacobianFcn', @constveljac, ...
            'PositionIndex', [1 3 5], ...
            'ExtentRotationFcn', @(x, dT)eye(3), ...
            'DegreesOfFreedom', dof, ...
            'ScaleMatrices', V, ...
            'TemporalDecay', 150, ...
            'Shapes', alpha, ...
            'Rates', beta, ...
            'GammaForgettingFactors', 1.05);
    end
end