function [radardets, radarTracks, inforadar] = updateRadarTracker(radar, radarPHD, targets, egoUAV, mountingLoc, mountingAngles, time)
    % 更新雷达跟踪器（基于官方示例）
    
    persistent lastRadarTime debugCount
    if isempty(lastRadarTime)
        lastRadarTime = 0;
        debugCount = 0;
    end
    
    % 初始化输出
    radardets = {};
    radarTracks = objectTrack.empty;
    inforadar = [];
    
    % 确保时间严格递增
    if time <= lastRadarTime
        time = lastRadarTime + 0.1;
    end
    
    debugCount = debugCount + 1;
    
    % 获取自车位姿
    try
        egoPose = read(egoUAV);
        egoPos = egoPose(1:3);
        egoVel = egoPose(4:6);
        
        quat_parts = egoPose(10:13);
        if size(quat_parts, 2) == 4
            egoOrient = quaternion(quat_parts);
        else
            egoOrient = quaternion(quat_parts');
        end
        
    catch ME
        if mod(debugCount, 100) == 1
            fprintf('[雷达错误] 无法读取自车位姿: %s\n', ME.message);
        end
        return;
    end
    
    % 计算雷达在世界坐标系中的位置和方向
    mountingQuat = quaternion(mountingAngles, 'eulerd', 'ZYX', 'frame');
    radarOrient = egoOrient * mountingQuat;
    
    if size(egoPos, 2) > 1
        egoPos = egoPos';
    end
    radarPos = egoPos + rotmat(egoOrient, 'frame') * mountingLoc(:);
    radarRotMat = rotmat(radarOrient, 'frame');
    
    % 手动生成检测（与之前相同）
    validTargets = 0;
    radardets = {};
    
    for i = 1:numel(targets)
        try
            pose = read(targets(i));
            
            if any(isnan(pose(1:6)))
                continue;
            end
            
            targetPos = pose(1:3);
            targetVel = pose(4:6);
            
            if size(targetPos, 2) > 1
                targetPos = targetPos';
            end
            if size(targetVel, 2) > 1
                targetVel = targetVel';
            end
            
            % 计算相对位置
            relPosWorld = targetPos - radarPos;
            distance = norm(relPosWorld);
            
            % 范围检查
            if distance < 5 || distance > 200
                continue;
            end
            
            % 转换到雷达坐标系
            relPosRadar = radarRotMat' * relPosWorld;
            
            % 视场检查
            azimuth = atan2d(relPosRadar(2), relPosRadar(1));
            elevation = asind(relPosRadar(3) / distance);
            
            if abs(azimuth) > 90 || abs(elevation) > 45
                continue;
            end
            
            % 添加噪声
            validTargets = validTargets + 1;
            rangeNoise = 2 * randn();
            azNoise = 0.5 * randn();
            elNoise = 0.5 * randn();
            
            noisyRange = distance + rangeNoise;
            noisyAz = azimuth + azNoise;
            noisyEl = elevation + elNoise;
            
            noisyX = noisyRange * cosd(noisyEl) * cosd(noisyAz);
            noisyY = noisyRange * cosd(noisyEl) * sind(noisyAz);
            noisyZ = noisyRange * sind(noisyEl);
            
            measurement = [noisyX; noisyY; noisyZ];
            
            % 创建检测
            radardets{end+1} = objectDetection(time, measurement, ...
                'MeasurementNoise', diag([2^2, 2^2, 2^2]), ...
                'ObjectClassID', i, ...
                'SensorIndex', 1); %#ok<AGROW>
            
        catch
            continue;
        end
    end
    
    % 调试输出
    if mod(debugCount, 100) == 1
        fprintf('[雷达调试] 时间=%.1f, 有效目标数=%d\n', time, validTargets);
        if validTargets > 0
            fprintf('  ✓ 生成检测数=%d\n', numel(radardets));
        else
            fprintf('  [雷达警告] 没有有效目标\n');
        end
    end
    
    % 更新传感器配置
    try
        configs = radarPHD.SensorConfigurations;
        if ~isempty(configs)
            if size(egoPos, 2) > 1
                egoPosCol = egoPos';
            else
                egoPosCol = egoPos;
            end
            if size(egoVel, 2) > 1
                egoVelCol = egoVel';
            else
                egoVelCol = egoVel;
            end
            
            configs{1}.SensorTransformParameters(2).OriginPosition = egoPosCol;
            configs{1}.SensorTransformParameters(2).OriginVelocity = egoVelCol;
            configs{1}.SensorTransformParameters(2).Orientation = rotmat(egoOrient, 'frame');
        end
    catch
        configs = radarPHD.SensorConfigurations;
    end
    
    % 执行跟踪更新
    try
        if ~isempty(radardets) && (isLocked(radarPHD) || ~isempty(radardets))
            if ~iscell(configs)
                configs = {configs};
            end
            
            [radarTracks, ~, ~, inforadar] = radarPHD(radardets, configs, time);
            lastRadarTime = time;
            
            if mod(debugCount, 100) == 1 && ~isempty(radarTracks)
                fprintf('  ✓ 雷达航迹数=%d\n', numel(radarTracks));
            end
            
        elseif isLocked(radarPHD)
            radarTracks = predictTracksToTime(radarPHD, 'confirmed', time);
        else
            radarTracks = objectTrack.empty;
        end
        
    catch ME
        if mod(debugCount, 100) == 1
            fprintf('  [跟踪错误] %s\n', ME.message);
        end
        radarTracks = objectTrack.empty;
    end
end