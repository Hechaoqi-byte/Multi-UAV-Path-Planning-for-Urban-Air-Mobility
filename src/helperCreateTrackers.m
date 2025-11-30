function [lidarDetector, lidarJPDA, radarPHD, gridTracker] = helperCreateTrackers(lidar, egoUAV, radarMountingLocation, radarMountingAngles)
% helperCreateTrackers Create and initialize all tracking components
%
% Description:
%   Initializes and configures multiple tracking components for a comprehensive
%   multi-sensor tracking system. Creates LiDAR detectors, JPDA tracker, 
%   radar PHD tracker, and grid-based tracker to handle different types of
%   sensor data and tracking scenarios.
%
% Input Parameters:
%   lidar                 - LiDAR sensor configuration object
%   egoUAV                - Ego UAV state information object
%   radarMountingLocation - Radar mounting position [x, y, z] relative to UAV
%   radarMountingAngles   - Radar mounting orientation [yaw, pitch, roll] in degrees
%
% Output Parameters:
%   lidarDetector - Custom LiDAR detector object for point cloud processing
%   lidarJPDA     - JPDA (Joint Probabilistic Data Association) tracker for LiDAR
%   radarPHD      - Radar PHD (Probability Hypothesis Density) tracker
%   gridTracker   - 3D grid-based tracker for occupancy grid processing
%
% Tracking Components:
%   1. LiDAR Detector: Custom point cloud processing for object detection
%   2. LiDAR JPDA Tracker: Multi-target tracking with probabilistic data association
%   3. Radar PHD Tracker: Multi-target tracking using probability hypothesis density
%   4. Grid Tracker: 3D grid-based tracking for occupancy maps
%

    % Create all trackers
    
    %% LiDAR Detector and Tracker
    % Use scene (empty scene here, will be updated during actual use)
    lidarDetector = customLidarDetector([]);
     % ===== Add this line (verify detector creation success) =====
    fprintf('  [Detector] Created successfully: %s\n', class(lidarDetector));
    % ========================================
    
    % JPDA tracker
    lidarJPDA = trackerJPDA;
    lidarJPDA.TrackerIndex = 2;
    lidarJPDA.FilterInitializationFcn = @initLidarFilter;
    lidarJPDA.TrackLogic = 'History';
    lidarJPDA.AssignmentThreshold = [70 150];
    lidarJPDA.ConfirmationThreshold = [4 5];
    lidarJPDA.DeletionThreshold = 10;
    lidarJPDA.ClutterDensity = 1e-6;
    lidarJPDA.DetectionProbability = 0.99;
    lidarJPDA.MaxNumTracks = 100;
    
    %% Radar PHD Tracker
    radarPHD = createRadarTracker(radarMountingLocation, radarMountingAngles, egoUAV);
    
    %% 3D Grid Tracker
    gridTracker = createGridTracker3D(lidar, egoUAV);
end