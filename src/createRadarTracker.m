function tracker = createRadarTracker(mountingLocation, mountingAngles, egoUAV)
% createRadarTracker Create radar PHD tracker based on official examples
%
% Description:
%   Creates a Probability Hypothesis Density (PHD) tracker specifically 
%   configured for radar sensor data processing. This tracker uses Gaussian 
%   mixture PHD filters to handle multiple target tracking in cluttered 
%   environments, with proper sensor modeling and coordinate transformations
%   for UAV applications.
%
% Input Parameters:
%   mountingLocation - Radar mounting position relative to UAV [x, y, z] (meters)
%   mountingAngles   - Radar mounting orientation [yaw, pitch, roll] (degrees)
%   egoUAV           - Ego UAV platform object containing state information
%
% Output Parameters:
%   tracker - Configured trackerPHD object for radar-based multi-target tracking
%
% Sensor Configuration:
%   Field of View: [180° azimuth, 90° elevation]
%   Resolution: [2° azimuth, 5° elevation, 2m range]
%   Range: 5-200 meters
%   Detection Probability: 0.98
%   False Alarm Rate: 1e-7
%
% Tracker Parameters:
%   Algorithm: Probability Hypothesis Density (PHD) with Gaussian mixtures
%   Birth Rate: 1e-3 - Rate of new target births
%   Assignment Threshold: 50 - Gate threshold for data association
%   Extraction Threshold: 0.80 - Threshold for track extraction from PHD
%   Confirmation Threshold: 0.99 - Probability threshold for track confirmation
%   Deletion Threshold: 0.1 - Probability threshold for track deletion
%   Max Number of Tracks: 100
%
% Coordinate Transformations:
%   - Sensor frame (spherical coordinates) to UAV frame
%   - UAV frame to scenario frame (rectangular coordinates)
%   - Handles elevation and proper orientation transformations

    % Create radar PHD tracker (based on official examples)
    % Input:
    %   mountingLocation - Radar mounting position [x y z]
    %   mountingAngles - Radar mounting angles [yaw pitch roll]
    %   egoUAV - Ego platform
    
    % Radar parameters (corresponding to configuration in main script)
    fov = [180 90];  % [azimuth elevation] - consistent with main script
    azimuthResolution = 2;
    elevationResolution = 5;
    rangeResolution = 2;
    falseAlarmRate = 1e-7;
    detectionProbability = 0.98;
    sensorIndex = 1;
    
    % Sensor limits (spherical coordinates)
    sensorLimits = [-fov(1)/2 fov(1)/2;      % Azimuth range (degrees)
                    -fov(2)/2 fov(2)/2;      % Elevation range (degrees)
                    5 200];                   % Range limits (meters)
    
    % Sensor resolution
    sensorResolution = [azimuthResolution; elevationResolution; rangeResolution];
    
    % Clutter density
    Kc = falseAlarmRate / (azimuthResolution * rangeResolution * elevationResolution);
    Pd = detectionProbability;
    
    % Sensor position and orientation
    sensorPos = mountingLocation(:);
    sensorOrient = rotmat(quaternion(mountingAngles, 'eulerd', 'ZYX', 'frame'), 'frame');
    
    % Sensor coordinate transformation relative to UAV (spherical coordinates)
    sensorTransformParameters(1) = struct(...
        'Frame', 'Spherical', ...
        'OriginPosition', sensorPos, ...
        'OriginVelocity', zeros(3,1), ...
        'Orientation', sensorOrient, ...
        'IsParentToChild', true, ...
        'HasElevation', true, ...
        'HasVelocity', false);
    
    % UAV coordinate transformation relative to scenario (rectangular coordinates)
    egoPose = read(egoUAV);
    sensorTransformParameters(2) = struct(...
        'Frame', 'Rectangular', ...
        'OriginPosition', egoPose(1:3), ...
        'OriginVelocity', egoPose(4:6), ...
        'Orientation', rotmat(quaternion(egoPose(10:13)), 'frame'), ...
        'IsParentToChild', true, ...
        'HasElevation', true, ...
        'HasVelocity', false);
    
    % Create sensor configuration
    radarPHDconfig = trackingSensorConfiguration(sensorIndex, ...
        'IsValidTime', true, ...
        'SensorLimits', sensorLimits, ...
        'SensorResolution', sensorResolution, ...
        'DetectionProbability', Pd, ...
        'ClutterDensity', Kc, ...
        'SensorTransformFcn', @cvmeas, ...
        'SensorTransformParameters', sensorTransformParameters);
    
    % Set filter initialization function
    radarPHDconfig.FilterInitializationFcn = @initRadarFilter;
    radarPHDconfig.MinDetectionProbability = 0.4;
    
    % Partition threshold
    threshold = [3 16];
    
    % Create PHD tracker
    tracker = trackerPHD(...
        'TrackerIndex', 1, ...
        'HasSensorConfigurationsInput', true, ...
        'SensorConfigurations', {radarPHDconfig}, ...
        'BirthRate', 1e-3, ...
        'AssignmentThreshold', 50, ...
        'ExtractionThreshold', 0.80, ...
        'ConfirmationThreshold', 0.99, ...
        'MergingThreshold', 50, ...
        'DeletionThreshold', 0.1, ...
        'LabelingThresholds', [1.01 0.01 0], ...
        'PartitioningFcn', @(dets) partitionDetections(dets, threshold(1), threshold(2), 'Distance', 'euclidean'), ...
        'MaxNumSensors', 1, ...
        'MaxNumTracks', 100);
end