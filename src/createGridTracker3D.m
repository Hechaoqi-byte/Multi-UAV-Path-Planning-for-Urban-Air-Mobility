function tracker = createGridTracker3D(lidar, egoUAV)
% createGridTracker3D Create 3D tracker using trackerGNN as replacement for trackerGridRFS3D
%
% Description:
%   Creates a 3D multi-object tracker using the Global Nearest Neighbor (GNN) 
%   algorithm as a replacement for the trackerGridRFS3D. This tracker is 
%   designed for processing 3D LiDAR point cloud data and tracking multiple 
%   dynamic objects in the environment.
%
% Input Parameters:
%   lidar  - LiDAR sensor configuration object (currently unused in implementation)
%   egoUAV - Ego UAV state information (currently unused in implementation)
%
% Output Parameters:
%   tracker - Configured trackerGNN object for 3D multi-object tracking
%
% Configuration Details:
%   - Sensor Configuration: Defines sensor limits, resolution, and detection properties
%   - Tracking Algorithm: Global Nearest Neighbor with Munkres assignment
%   - Filter Initialization: Custom lidar filter for state estimation
%   - Track Management: Confirmation and deletion thresholds for track lifecycle
%
% Key Parameters:
%   AssignmentThreshold: [50 100] - Gate thresholds for assignment
%   ConfirmationThreshold: [3 5] - Hits required for track confirmation
%   DeletionThreshold: [5 5] - Misses allowed before track deletion
%   MaxNumTracks: 100 - Maximum number of maintained tracks

    % Create 3D tracker (using trackerGNN as replacement for trackerGridRFS3D)
    
    fprintf('[INFO] Using trackerGNN as replacement for trackerGridRFS3D\n');
    
    % Create sensor configuration
    sensorConfig = trackingSensorConfiguration('SensorIndex', 1, ...
        'IsValidTime', true, ...
        'SensorLimits', [-100 100; -100 100; -10 10], ...
        'SensorResolution', [2; 2; 5], ...
        'DetectionProbability', 0.95, ...
        'ClutterDensity', 1e-6);
    
    % Use trackerGNN (Global Nearest Neighbor tracker)
    tracker = trackerGNN(...
        'TrackerIndex', 3, ...
        'FilterInitializationFcn', @initLidarFilter, ...
        'Assignment', 'Munkres', ...
        'AssignmentThreshold', [50 100], ...
        'ConfirmationThreshold', [3 5], ...
        'DeletionThreshold', [5 5], ...
        'MaxNumTracks', 100, ...
        'MaxNumSensors', 1);
end