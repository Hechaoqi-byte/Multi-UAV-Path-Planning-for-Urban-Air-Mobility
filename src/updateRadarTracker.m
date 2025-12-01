function [radardets, radarTracks, inforadar] = updateRadarTracker(radar, radarPHD, targets, egoUAV, mountingLoc, mountingAngles, time)
    % Update radar tracker 
    
    persistent lastRadarTime debugCount
    if isempty(lastRadarTime)
        lastRadarTime = 0;
        debugCount = 0;
    end
    
    % Initialize outputs
    radardets = {};
    radarTracks = objectTrack.empty;
    inforadar = [];
    
    % Ensure strictly increasing time
    if time <= lastRadarTime
        time = lastRadarTime + 0.1;
    end
    
    debugCount = debugCount + 1;
    
    % Get ego vehicle pose
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
            fprintf('[Radar Error] Unable to read ego pose: %s\n', ME.message);
        end
        return;
    end
    
    % Calculate radar position and orientation in world coordinates
    mountingQuat = quaternion(mountingAngles, 'eulerd', 'ZYX', 'frame');
    radarOrient = egoOrient * mountingQuat;
    
    if size(egoPos, 2) > 1
        egoPos = egoPos';
    end
    radarPos = egoPos + rotmat(egoOrient, 'frame') * mountingLoc(:);
    radarRotMat = rotmat(radarOrient, 'frame');
    
    % Manual detection generation (same as before)
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
            
            % Calculate relative position
            relPosWorld = targetPos - radarPos;
            distance = norm(relPosWorld);
            
            % Range check
            if distance < 5 || distance > 200
                continue;
            end
            
            % Transform to radar coordinate system
            relPosRadar = radarRotMat' * relPosWorld;
            
            % Field of view check
            azimuth = atan2d(relPosRadar(2), relPosRadar(1));
            elevation = asind(relPosRadar(3) / distance);
            
            if abs(azimuth) > 90 || abs(elevation) > 45
                continue;
            end
            
            % Add noise
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
            
            % Create detection
            radardets{end+1} = objectDetection(time, measurement, ...
                'MeasurementNoise', diag([2^2, 2^2, 2^2]), ...
                'ObjectClassID', i, ...
                'SensorIndex', 1); %#ok<AGROW>
            
        catch
            continue;
        end
    end
    
    % Debug output
    if mod(debugCount, 100) == 1
        fprintf('[Radar Debug] Time=%.1f, Valid targets=%d\n', time, validTargets);
        if validTargets > 0
            fprintf('  ✓ Generated detections=%d\n', numel(radardets));
        else
            fprintf('  [Radar Warning] No valid targets\n');
        end
    end
    
    % Update sensor configuration
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
    
    % Execute tracking update
    try
        if ~isempty(radardets) && (isLocked(radarPHD) || ~isempty(radardets))
            if ~iscell(configs)
                configs = {configs};
            end
            
            [radarTracks, ~, ~, inforadar] = radarPHD(radardets, configs, time);
            lastRadarTime = time;
            
            if mod(debugCount, 100) == 1 && ~isempty(radarTracks)
                fprintf('  ✓ Radar tracks=%d\n', numel(radarTracks));
            end
            
        elseif isLocked(radarPHD)
            radarTracks = predictTracksToTime(radarPHD, 'confirmed', time);
        else
            radarTracks = objectTrack.empty;
        end
        
    catch ME
        if mod(debugCount, 100) == 1
            fprintf('  [Tracking Error] %s\n', ME.message);
        end
        radarTracks = objectTrack.empty;
    end
end