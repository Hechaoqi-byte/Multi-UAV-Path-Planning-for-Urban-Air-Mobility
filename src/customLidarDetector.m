classdef customLidarDetector < handle
    % Custom LiDAR Detector - Extract targets from point cloud
    
    properties
        Scene                    % UAV scene
        MinPoints = 15           % Minimum number of points
        DistanceThreshold = 3.0  % Clustering distance threshold (meters)
        MaxDetections = 20       % Maximum number of detections
        MinClusterSize = 15      % Minimum cluster size
        MaxClusterSize = 500     % Maximum cluster size
    end
    
    methods
        function obj = customLidarDetector(scene)
            % Constructor
            obj.Scene = scene;
        end
        
        function [detections, boxCenters] = detect(obj, pointCloud, time)
            % Detect targets from point cloud
            
            detections = {};
            boxCenters = [];
            
            % Check point cloud validity
            if isempty(pointCloud) || pointCloud.Count < obj.MinPoints
                return;
            end
            
            try
                % Get point cloud data
                points = pointCloud.Location;
                
                % Remove ground points (assuming ground at z < -5)
                validIdx = points(:,3) > -5;
                points = points(validIdx, :);
                
                if size(points, 1) < obj.MinPoints
                    return;
                end
                
                % Distance-based clustering
                [labels, numClusters] = obj.clusterPoints(points);
                
                if numClusters == 0
                    return;
                end
                
                % Create detection for each cluster
                detIdx = 1;
                for i = 1:numClusters
                    clusterPoints = points(labels == i, :);
                    numPts = size(clusterPoints, 1);
                    
                    % Filter too small or too large clusters
                    if numPts < obj.MinClusterSize || numPts > obj.MaxClusterSize
                        continue;
                    end
                    
                    % Calculate cluster center
                    center = mean(clusterPoints, 1)';
                    
                    % Calculate covariance (as measurement noise)
                    if numPts > 1
                        cov_val = cov(clusterPoints);
                        measNoise = diag([max(cov_val(1,1), 0.5), ...
                                         max(cov_val(2,2), 0.5), ...
                                         max(cov_val(3,3), 0.5)]);
                    else
                        measNoise = eye(3) * 1.0;
                    end
                    
                    % Create detection object
                    det = objectDetection(time, center, ...
                        'MeasurementNoise', measNoise, ...
                        'ObjectClassID', 1, ...
                        'SensorIndex', 2);
                    
                    detections{detIdx} = det; %#ok<AGROW>
                    boxCenters = [boxCenters; center']; %#ok<AGROW>
                    detIdx = detIdx + 1;
                    
                    if detIdx > obj.MaxDetections
                        break;
                    end
                end
                
            catch ME
                warning('LiDAR detection failed: %s', ME.message);
                detections = {};
                boxCenters = [];
            end
        end
        
        function [labels, numClusters] = clusterPoints(obj, points)
            % Point cloud clustering based on distance
            
            numPoints = size(points, 1);
            
            if numPoints < obj.MinPoints
                labels = [];
                numClusters = 0;
                return;
            end
            
            try
                % Use hierarchical clustering
                if numPoints > 5000
                    % Too many points, random sampling
                    idx = randperm(numPoints, 5000);
                    sampledPoints = points(idx, :);
                else
                    sampledPoints = points;
                end
                
                % Calculate distance matrix and cluster
                distances = pdist(sampledPoints, 'euclidean');
                Z = linkage(distances, 'single');
                labels_sampled = cluster(Z, 'cutoff', obj.DistanceThreshold, 'criterion', 'distance');
                
                % If sampled, need to map labels back to original points
                if numPoints > 5000
                    labels = zeros(numPoints, 1);
                    labels(idx) = labels_sampled;
                    % Assign nearest cluster for unsampled points
                    unsampled = setdiff(1:numPoints, idx);
                    for i = unsampled
                        [~, nearestIdx] = min(vecnorm(sampledPoints - points(i,:), 2, 2));
                        labels(i) = labels_sampled(nearestIdx);
                    end
                else
                    labels = labels_sampled;
                end
                
                numClusters = max(labels);
                
            catch ME
                warning('Clustering failed: %s', ME.message);
                labels = ones(numPoints, 1);
                numClusters = 1;
            end
        end
    end
end