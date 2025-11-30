function [gridTracks, gridMap] = updateGridTracker3D(lidar, gridTracker, egoUAV, time)
    % 更新3D栅格跟踪器（使用GNN跟踪器）
    persistent lastGridTime
    if isempty(lastGridTime)
        lastGridTime = 0;
    end

    % 确保时间严格递增
    if time <= lastGridTime
        time = lastGridTime + 0.1;
    end

    % 检查时间递增
    if time <= lastGridTime
        fprintf('[警告] 栅格跟踪器时间未递增: %.6f -> %.6f\n', lastGridTime, time);
        if isLocked(gridTracker)
            gridTracks = predictTracksToTime(gridTracker, 'confirmed', time);
        else
            gridTracks = objectTrack.empty;
        end
        gridMap = [];
        return;
    end

    % 读取点云数据
    [~, ~, ptCloud] = read(lidar);
    
    % 获取ego位姿
    egoPose = read(egoUAV);
    
    % 从点云生成检测
    detections = {};
    if ~isempty(ptCloud) && ptCloud.Count > 10
        try
            % 简单聚类检测
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
            fprintf('[警告] 点云聚类失败: %s\n', ME.message);
        end
    end

    % 执行跟踪更新
    try
        if isLocked(gridTracker) || ~isempty(detections)
            gridTracks = gridTracker(detections, time);
            lastGridTime = time;
            
            if mod(round(time*10), 50) == 0
                fprintf('[信息] 3D栅格(GNN): 航迹数=%d\n', numel(gridTracks));
            end
        else
            gridTracks = objectTrack.empty;
        end
        
        gridMap = [];  % GNN不输出地图
        
    catch ME
        fprintf('[错误] 3D栅格跟踪器更新失败: %s\n', ME.message);
        if isLocked(gridTracker)
            gridTracks = predictTracksToTime(gridTracker, 'confirmed', time);
        else
            gridTracks = objectTrack.empty;
        end
        gridMap = [];
    end
end