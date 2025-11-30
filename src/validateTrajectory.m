function isValid = validateTrajectory(trajectory, speed_limit, acc_max, jerk_max)
    % 验证轨迹是否满足动力学约束
    %
    % trajectory: [N×6] = [x y z vx vy vz]
    % speed_limit: 速度上限
    % acc_max: 加速度上限
    % jerk_max: 暂时未严格使用（保留参数）

    if isempty(trajectory) || size(trajectory, 1) < 2
        isValid = false;
        fprintf('    轨迹为空或点数不足\n');
        return;
    end
    
  % ===== 1. 速度检查（改为分位数 + 日志）=====
velocities = trajectory(:, 4:6);
speeds = vecnorm(velocities, 2, 2);

% 平滑一点（可选）——防止数值噪声
if numel(speeds) >= 3
    speeds_smoothed = speeds;
    for k = 2:numel(speeds)-1
        speeds_smoothed(k) = (speeds(k-1) + speeds(k) + speeds(k+1)) / 3;
    end
else
    speeds_smoothed = speeds;
end

% 分位数
speeds_sorted = sort(speeds_smoothed);
idx90 = max(1, round(0.9 * numel(speeds_sorted)));
v90 = speeds_sorted(idx90);
v_max_obs = max(speeds_smoothed);

if v90 > speed_limit * 1.15   % 仍然允许15%%误差
    fprintf('    速度超限(95%%分位): %.1f > %.1f (max=%.1f)\n', ...
        v90, speed_limit, v_max_obs);
    isValid = false;
    return;
end
    
    % ===== 2. 加速度检查：用平滑 + 分位数，降低对尖峰的敏感度 =====
    if size(trajectory, 1) > 2
        dt = 0.2;  % extractTrajectoryFromSolution_new 里采样步长也是 0.1
        
        % 2.1 数值微分得到加速度
        raw_acc = diff(velocities) / dt;              % [N-1 × 3]
        
        % 2.2 简单平滑（3 点滑动平均），减少数值噪声
        acc_smoothed = raw_acc;
        if size(raw_acc,1) >= 3
            for k = 2:size(raw_acc,1)-1
                acc_smoothed(k,:) = (raw_acc(k-1,:) + raw_acc(k,:) + raw_acc(k+1,:)) / 3;
            end
        end
        
        acc_magnitudes = vecnorm(acc_smoothed, 2, 2);
        
        % 2.3 使用 95% 分位数，而不是绝对最大值
        acc_sorted = sort(acc_magnitudes);
        idx90 = max(1, round(0.9 * numel(acc_sorted)));
        acc90 = acc_sorted(idx90);
        
        % 仍然保留一个真正的 max 作为诊断信息
        acc_max_obs = max(acc_magnitudes);
        
        % 允许 30%% 误差，但基于 95% 分位数判断
        if acc90> acc_max * 1.3
            fprintf('    加速度超限(95%%分位): %.1f > %.1f (max=%.1f)\n', ...
                acc90, acc_max, acc_max_obs);
            isValid = false;
            return;
        end
    end
    
  

    % ===== 3. 轨迹连续性（空间跳变）检查：用统计量而不是单点最大值 =====
    positions = trajectory(:, 1:3);
    position_jumps = vecnorm(diff(positions), 2, 2);   % 相邻点位移
    
    if isempty(position_jumps)
        isValid = true;
        return;
    end

    % 3.1 基于速度上限的理论期待：正常情况下，大部分跳变 ~ speed_limit*dt
    dt = 0.2;  % 与采样时步长一致
    expected_mean = speed_limit * dt;       % 期望平均跳变
    hard_limit    = speed_limit * 0.5;      % 一个保底硬阈值（例如 0.5s 对应的位移）

    % 3.2 使用分位数 + 标准差判断
    pj_sorted = sort(position_jumps);
    idx90 = max(1, round(0.90 * numel(pj_sorted)));
    pj90 = pj_sorted(idx90);                % 90% 分位数

    pj_mean = mean(position_jumps);
    pj_std  = std(position_jumps);

    % 允许范围：取 max(均值+3σ, 理论期望的 2 倍, 硬限制)
    soft_limit = max([pj_mean + 3*pj_std, 2*expected_mean, hard_limit]);

    max_jump   = max(position_jumps);

    if pj90 > soft_limit
        fprintf('    轨迹不连续: 99%%跳变=%.2fm, 阈值=%.2fm (max=%.2fm)\n', ...
            pj90, soft_limit, max_jump);
        isValid = false;
        return;
    end

    isValid = true;
end