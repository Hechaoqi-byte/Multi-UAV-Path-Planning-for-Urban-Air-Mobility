function key_points = selectKeyPointsByCurvature(path, current_pos, max_points)
    % Select key points from BiRRT path (start/end points + high curvature points + uniform supplementary points)

    % ===== Input Dimension Correction =====
    if size(current_pos, 1) > size(current_pos, 2)
        current_pos = current_pos';
    end
    if size(path, 2) ~= 3 && size(path, 1) == 3
        path = path';
    end

    if isempty(path) || size(path, 1) < 2
        key_points = [current_pos; path];
        return;
    end

    fprintf('[Key Point Selection] Processing %d path points\n', size(path, 1));

    % 1. Find the path point closest to current position as starting point
    distances_to_current = vecnorm(path - current_pos, 2, 2);
    [~, start_idx] = min(distances_to_current);

    % Extract path segment starting from current position
    path_segment = path(start_idx:end, :);

    if size(path_segment, 1) < 3
        key_points = [current_pos; path_segment];
        return;
    end

    % 2. Calculate path curvature (polyline turning angles)
    curvatures = calculatePathCurvature(path_segment);

    % 3. Mandatory points: start point + end point
    mandatory_indices = [1, size(path_segment, 1)];

    % 4. High curvature points (> 30°)
    curvature_threshold = pi/6; % 30 degrees
    high_curvature_indices = find(curvatures > curvature_threshold);

    % 5. Merge candidate points
    candidate_indices = unique([mandatory_indices, high_curvature_indices']);

    % 5.5 Ensure at least one intermediate candidate point (prevent only start/end)
    if numel(candidate_indices) < 3
        mid_idx = round(size(path_segment,1) / 2);
        candidate_indices = unique([candidate_indices, mid_idx]);
    end

    % 6. If too many candidate points, select top max_points by curvature
    if numel(candidate_indices) > max_points
        curvature_scores = curvatures(candidate_indices);
        [~, sorted_idx] = sort(curvature_scores, 'descend');
        % Start and end points must be kept
        must_keep = [1, size(path_segment, 1)];
        must_keep_mask = ismember(candidate_indices, must_keep);
        % Select from high curvature first, fill remaining slots by curvature priority
        selected_idx = candidate_indices(must_keep_mask);
        remaining = max_points - numel(selected_idx);
        if remaining > 0
            others = candidate_indices(~must_keep_mask);
            others_scores = curvatures(others);
            [~, o_sorted] = sort(others_scores, 'descend');
            selected_idx = [selected_idx, others(o_sorted(1:min(remaining, numel(others))))];
        end
        selected_indices = unique(selected_idx);
    else
        selected_indices = candidate_indices;
    end

    % 7. If still too few points, uniformly supplement points to reach "minimum number"
    min_points = min(5, max_points);  % At least 5 points (including start/end), adjustable as needed
    if numel(selected_indices) < min_points
        % Uniformly select min_points indices from entire path_segment
        uniform_idx = round(linspace(1, size(path_segment,1), min_points));
        selected_indices = unique([selected_indices, uniform_idx]);
        % Truncate to max_points if needed
        if numel(selected_indices) > max_points
            tmp = round(linspace(1, numel(selected_indices), max_points));
            selected_indices = selected_indices(tmp);
        end
    end

    % 8. Sort by path order
    selected_indices = sort(selected_indices);

    % 9. Extract key points
    key_points_from_path = path_segment(selected_indices, :);

    % 10. Ensure current position is included
    if norm(key_points_from_path(1,:) - current_pos) > 0.1
        key_points = [current_pos; key_points_from_path];
    else
        key_points = key_points_from_path;
    end

    % 11. Final hard limit max_points
    if size(key_points,1) > max_points
        indices = round(linspace(1, size(key_points,1), max_points));
        key_points = key_points(indices,:);
    end

    fprintf('  Selected %d key points (from %d path points)\n', size(key_points, 1), size(path_segment, 1));
    if size(key_points, 1) > 2
        key_curvatures = calculatePathCurvature(key_points);
        max_curvature = max(key_curvatures);
        fprintf('  Maximum curvature: %.1f degrees\n', rad2deg(max_curvature));
    end
end

%% ------------------------------------------------------------
%% Subfunction: Calculate Path Curvature (Polyline Turning Angles)
%% ------------------------------------------------------------
function curvatures = calculatePathCurvature(path)
    % Calculate "turning angle" at each path point (three-point angle)

    n = size(path, 1);
    curvatures = zeros(n, 1);

    for i = 2:n-1
        p1 = path(i-1, :);
        p2 = path(i, :);
        p3 = path(i+1, :);

        v1 = p2 - p1;
        v2 = p3 - p2;

        if norm(v1) > 0.1 && norm(v2) > 0.1
            v1_norm = v1 / norm(v1);
            v2_norm = v2 / norm(v2);

            dot_product = dot(v1_norm, v2_norm);
            dot_product = max(-1, min(1, dot_product)); % Prevent numerical errors
            angle = acos(dot_product);                  % Radians [0,π]

            curvatures(i) = angle;
        else
            curvatures(i) = 0;
        end
    end

    % Set start and end points to 0
    curvatures(1)   = 0;
    curvatures(end) = 0;
end