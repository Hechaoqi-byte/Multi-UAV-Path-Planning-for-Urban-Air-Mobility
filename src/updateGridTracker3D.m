function [gridTracks, gridMap] = updateGridTracker3D(lidar, gridTracker, egoUAV, time)
% updateGridTracker3D Update 3D grid tracker using GNN tracker
%
% Description:
%   Updates a 3D grid-based tracker using Global Nearest Neighbor (GNN) algorithm.
%   Processes LiDAR point cloud data to generate detections, performs clustering,
%   and updates the tracker state. Handles time synchronization and provides
%   robust error handling for real-time tracking applications.
%
% Input Parameters:
%   lidar       - LiDAR sensor object for point cloud data acquisition
%   gridTracker - GNN tracker object for multi-target tracking
%   egoUAV      - Ego UAV platform object for pose information
%   time        - Current simulation time (must be strictly increasing)
%
% Output Parameters:
%   gridTracks  - Array of updated objectTrack objects containing:
%                 .TrackID     - Unique track identifier
%                 .State       - State vector [x,vx,y,vy,z,vz]
%                 .StateCovariance - State uncertainty covariance
%                 .ObjectClassID - Object classification
%   gridMap     - Empty output (reserved for future grid map functionality)
%
% Algorithm:
%   1. Time synchronization and validation (ensures strictly increasing time)
%   2. LiDAR point cloud acquisition and preprocessing
%   3. Point cloud clustering using Euclidean distance clustering
%   4. Detection generation from cluster centers
%   5. Tracker update with generated detections
%   6. Robust error handling and fallback mechanisms
%
% Features:
%   - Strict time increment enforcement for tracker stability
%   - Euclidean distance clustering for detection generation
%   - Comprehensive error handling and fallback strategies
%   - Periodic status reporting for monitoring
%   - Support for both initialized and uninitialized tracker states

    % Update 3D grid tracker (using GNN tracker)
    persistent lastGridTime
    if isempty(lastGridTime)
        lastGridTime = 0;
    end

    % Ensure time strictly increases
    if time <= lastGridTime
        time = lastGridTime + 0.1;
    end

    % Check time increment
    if time <= lastGridTime
        fprintf('[Warning] Grid tracker time not increasing: %.6f -> %.6f\n', lastGridTime, time);
        if isLocked(gridTracker)
            gridTracks = predictTracksToTime(gridTracker, 'confirmed', time);
        else
            gridTracks = objectTrack.empty;
        end
        gridMap = [];
        return;
    end

    % Read point cloud data
    [~, ~, ptCloud] = read(lidar);
    
    % Get ego pose
    egoPose = read(egoUAV);
    
    % Generate detections from point cloud
    detections = {};
    if ~isempty(ptCloud) && ptCloud.Count > 10
        try
            % Simple clustering detection
            [labels, numClusters] = pcsegdist(ptCloud, 5);
            
            for i = 1:numClusters
                clusterIdx = (labels == i);
                clusterPts = select(ptCloud, clusterIdx);
                
                if clusterPts.Count > 5
                    center = mean(clusterPts.Location, 1);
                    
                    det = objectDetection(time, center', ...
                        'MeasurementNoise', diag([2 2 2]), ...
                        'ObjectClassID', 1);
                    detections{end+1} = det;
                end
            end
        catch ME
            fprintf('[Warning] Point cloud clustering failed: %s\n', ME.message);
        end
    end

    % Execute tracking update
    try
        if isLocked(gridTracker) || ~isempty(detections)
            gridTracks = gridTracker(detections, time);
            lastGridTime = time;
            
            if mod(round(time*10), 50) == 0
                fprintf('[Info] 3D Grid (GNN): Track count=%d\n', numel(gridTracks));
            end
        else
            gridTracks = objectTrack.empty;
        end
        
        gridMap = [];  % GNN does not output map
        
    catch ME
        fprintf('[Error] 3D grid tracker update failed: %s\n', ME.message);
        if isLocked(gridTracker)
            gridTracks = predictTracksToTime(gridTracker, 'confirmed', time);
        else
            gridTracks = objectTrack.empty;
        end
        gridMap = [];
    end
end