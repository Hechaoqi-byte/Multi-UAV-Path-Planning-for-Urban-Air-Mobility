function [isValid, collisionProb] = validateTrajectories(validator, trajectories)
    % 验证轨迹是否与障碍物碰撞
    
    numTrajs = length(trajectories);
    isValid = true(numTrajs, 1);
    collisionProb = zeros(numTrajs, 1);
    
    if ~isfield(validator, 'predictedObstacles') || isempty(validator.predictedObstacles)
        return;
    end
    
    if isfield(validator, 'safetyMargin')
        safetyMargin = validator.safetyMargin;
    else
        safetyMargin = 10;
    end
    
    for i = 1:numTrajs
        traj = trajectories{i};
        
        if isempty(traj) || size(traj, 1) < 2
            isValid(i) = false;
            collisionProb(i) = 1.0;
            continue;
        end
        
        trajPos = traj(:, 1:3);
        minDist = inf;
        
        for j = 1:length(validator.predictedObstacles)
            obsPred = validator.predictedObstacles{j};
            
            if isempty(obsPred)
                continue;
            end
            
            numSteps = min(size(trajPos, 1), size(obsPred, 1));
            
            for t = 1:numSteps
                dist = norm(trajPos(t, :) - obsPred(t, :));
                minDist = min(minDist, dist);
            end
            
            % 检查中间点
            if numSteps > 1
                for t = 1:numSteps-1
                    for alpha = 0.25:0.25:0.75
                        interpTraj = trajPos(t,:) * (1-alpha) + trajPos(t+1,:) * alpha;
                        t2 = min(t+1, size(obsPred,1));
                        interpObs = obsPred(t,:) * (1-alpha) + obsPred(t2,:) * alpha;
                        dist = norm(interpTraj - interpObs);
                        minDist = min(minDist, dist);
                    end
                end
            end
        end
        
        % ★★★ 修复：更细致的碰撞概率 ★★★
        if minDist < safetyMargin * 0.3
            isValid(i) = false;
            collisionProb(i) = 1.0;
        elseif minDist < safetyMargin * 0.6
            isValid(i) = false;
            collisionProb(i) = 0.8 + 0.2 * (1 - (minDist - safetyMargin*0.3) / (safetyMargin*0.3));
        elseif minDist < safetyMargin
            isValid(i) = false;
            collisionProb(i) = 0.4 + 0.4 * (1 - (minDist - safetyMargin*0.6) / (safetyMargin*0.4));
        elseif minDist < safetyMargin * 1.5
            isValid(i) = true;
            collisionProb(i) = 0.15 + 0.25 * (1 - (minDist - safetyMargin) / (safetyMargin*0.5));
        elseif minDist < safetyMargin * 2.0
            isValid(i) = true;
            collisionProb(i) = 0.05 + 0.1 * (1 - (minDist - safetyMargin*1.5) / (safetyMargin*0.5));
        else
            isValid(i) = true;
            collisionProb(i) = 0;
        end
    end
end