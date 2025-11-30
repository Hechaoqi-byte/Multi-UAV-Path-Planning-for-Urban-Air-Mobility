function planner = updatePlannerReference(planner, newTrajectory) 
    % 更新规划器的参考路径
    
    if isfield(planner, 'refPath')
        % 使用我们新建的通用函数来转换结构体
        refPath_new = convertToRefPathStruct(newTrajectory(:, 1:3));
        
        % 替换旧的refPath
        planner.refPath = refPath_new;
        
        % fprintf('    规划器参考路径已更新 (%.1fm)\n', refPath_new.sMax);
    else
        warning('规划器结构不兼容，无法更新参考路径');
    end
end