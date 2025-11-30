function [lidardets, lidarTracks, boxCenters, infolidar] = updateLidarTracker(lidar, lidarDetector, lidarJPDA, egoPose, time)
% updateLidarTracker Update LiDAR tracker with debugging capabilities
%
% Description:
%   Updates the LiDAR tracking system by processing point cloud data, 
%   generating detections, and updating the JPDA tracker. Includes comprehensive
%   debugging features, error handling, and performance optimizations for
%   real-time operation. Enforces detection limits to maintain tracker stability.
%
% Input Parameters:
%   lidar        - LiDAR sensor object for point cloud data acquisition
%   lidarDetector- Custom LiDAR detector object for point cloud processing
%   lidarJPDA    - JPDA tracker object for multi-target tracking
%   egoPose      - Ego vehicle pose information (currently unused)
%   time         - Current simulation time for tracking update
%
% Output Parameters:
%   lidardets    - Cell array of LiDAR detections generated from point cloud
%   lidarTracks  - Array of updated objectTrack objects from JPDA tracker
%   boxCenters   - Detection bounding box centers [N×3] for visualization
%   infolidar    - Additional tracker information (assignment details, etc.)
%
% Algorithm:
%   1. Input validation and detector availability check
%   2. Point cloud data acquisition and preprocessing
%   3. Target detection using custom LiDAR detector
%   4. Detection count limitation for tracker stability (max 5 detections)
%   5. JPDA tracker update with filtered detections
%   6. Comprehensive error handling and debugging output
%
% Features:
%   - Strict detection count limitation (max 5) for tracker stability
%   - Periodic debugging output every 100 cycles
%   - Robust error handling for sensor and tracker failures
%   - Support for both initialized and uninitialized tracker states
%   - Time-based performance monitoring

    % Update LiDAR tracker (with debugging)
    
    persistent lastLidarTime debugCount
    if isempty(lastLidarTime)
        lastLidarTime = 0;
        debugCount = 0;
    end
    
    % Initialize outputs
    lidardets = {};
    lidarTracks = objectTrack.empty;
    boxCenters = [];
    infolidar = [];
    % ===== Add here (after line 16) =====
    if isempty(lidarDetector) || ~isobject(lidarDetector)
        try
            if isLocked(lidarJPDA)
                [lidarTracks, ~, ~, infolidar] = lidarJPDA([], time);
            end
        catch
        end
        debugCount = debugCount + 1;
        return;
    end
    % ================================
    
    % Ensure time strictly increases
     %time = lastLidarTime + 0.1;
    %end
    
    % Read LiDAR data
    try
        [isUpdated, ~, pointCloud] = lidar.read();
        
        debugCount = debugCount + 1;
        if mod(debugCount, 100) == 1
            fprintf('[LiDAR Debug] Time=%.1f, Updated=%d', time, isUpdated);
            if isUpdated && ~isempty(pointCloud)
                fprintf(', Point count=%d\n', pointCloud.Count);
            else
                fprintf(', No point cloud data\n');
            end
        end
        
        if isUpdated && ~isempty(pointCloud) && pointCloud.Count > 0
            % Detect targets
            [lidardets, boxCenters] = lidarDetector.detect(pointCloud, time);
            
            if mod(debugCount, 100) == 1
                fprintf('  Detections=%d\n', numel(lidardets));
            end
        end
    catch ME
        if mod(debugCount, 100) == 1
            fprintf('[LiDAR Error] %s\n', ME.message);
        end
        lidardets = {};
        boxCenters = [];
    end
    
    % Update tracker
    try
        if isLocked(lidarJPDA) || ~isempty(lidardets)
             % ===== Force limit detections ≤5 (critical!) =====
            if numel(lidardets) > 5
                lidardets = lidardets(1:5);
                if mod(debugCount, 100) == 1
                    fprintf('  [Force Limit] Detections→5\n');
                end
            end
            % ====================================
            [lidarTracks, ~, ~, infolidar] = lidarJPDA(lidardets, time);
            lastLidarTime = time;
            
            if mod(debugCount, 100) == 1 && ~isempty(lidarTracks)
                fprintf('  LiDAR tracks=%d\n', numel(lidarTracks));
            end
        else
            lidarTracks = objectTrack.empty;
        end
    catch ME
        if mod(debugCount, 100) == 1
            fprintf('[Tracker Error] %s\n', ME.message);
        end
        lidarTracks = objectTrack.empty;
    end
end