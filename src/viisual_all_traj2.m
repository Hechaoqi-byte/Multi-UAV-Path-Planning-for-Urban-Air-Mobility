%% Multi-UAV Simulation Trajectory Animation with Urban Environment
% 
% Description:
%   Creates a comprehensive 3D animation of multiple UAV trajectories in an
%   urban environment. The script automatically loads trajectory data from
%   result files, visualizes buildings, charging stations, task points, and
%   animates UAV movements in real-time. Supports dynamic playback with
%   proper timing synchronization across all UAVs.
%
% Input Files:
%   traj_result_uavXX.mat - Trajectory result files for each UAV containing:
%                           xtraj: trajectory data [N×M] (positions + states)
%                           ttraj: time vector [N×1]
%
% Global Variables Required (from base workspace):
%   buildingMeshes - Structure array containing building mesh data
%   charger_pos    - Charging station positions [P×3]
%   tasks_3d       - Task point positions [Q×3]
%
% Output:
%   3D animation figure displaying:
%   - Urban building meshes with colored elevation
%   - Charging stations (blue squares with labels)
%   - Task points (red triangles with labels)
%   - UAV trajectories (colored lines with real-time animation)
%   - Current UAV positions (colored spheres)
%   - Final UAV positions (colored diamonds)
%   - Comprehensive legend
%
% Animation Features:
%   - Real-time trajectory playback with synchronized timing
%   - Smooth animation with configurable frame rate
%   - Color-coded UAV identification
%   - Progressive trajectory drawing
%   - Final position markers

%% 1. Automatically collect & load all traj_result_uavXX.mat files
files = dir('traj_result_uav*.mat');
uav_list = [];
pos_all = {}; t_all = {};
for k = 1:numel(files)
    fname = files(k).name;
    expr = 'traj_result_uav(\d+)\.mat';
    tokens = regexp(fname, expr, 'tokens');
    if ~isempty(tokens)
        uav_id = str2double(tokens{1}{1});
        uav_list(end+1) = uav_id;
        load(fname);   % Load xtraj, ttraj
        if iscell(xtraj)
            pos_all{end+1} = xtraj{1}(:,1:3);
            t_all{end+1} = ttraj{1};
        else
            pos_all{end+1} = xtraj(:,1:3);
            t_all{end+1} = ttraj;
        end
    end
end
[~, idx_sort] = sort(uav_list);   % Sort by UAV ID
uav_list = uav_list(idx_sort); pos_all = pos_all(idx_sort); t_all = t_all(idx_sort);
n_uav = numel(uav_list);
colors = lines(max(n_uav,7));

%% 2. Draw city mesh
figure('Color','w');
hold on
axis equal; grid on; view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Multi-UAV Simulation Trajectory Dynamic Playback (with City Mesh)');

%% ==== 1. Building Mesh (light background) ====
if evalin('base','exist(''buildingMeshes'',''var'')')
    buildingMeshes = evalin('base','buildingMeshes');
    zmins = zeros(1,length(buildingMeshes));
    zmaxs = zeros(1,length(buildingMeshes));
    all_verts = [];
    for i = 1:length(buildingMeshes)
        verts = buildingMeshes(i).Vertices;
        zmins(i) = min(verts(:,3));
        zmaxs(i) = max(verts(:,3));
        all_verts = [all_verts; verts];
    end
    global_zmin = min(zmins);
    global_zmax = max(zmaxs);

    % ==== Ground area boundary ====
    xMin = min(all_verts(:,1));
    xMax = max(all_verts(:,1));
    yMin = min(all_verts(:,2));
    yMax = max(all_verts(:,2));

    % ==== Draw green ground ====
    fill3([xMin xMax xMax xMin], [yMin yMin yMax yMax], [0 0 0 0], ...
        [0.82 1 0.82], ... % Light green
        'FaceAlpha', 0.63, 'EdgeColor', 'none');

    % ==== Building mesh ====
    baseColor = [0.92 0.92 0.97];
    topColor  = [0.7 0.78 0.92];
    for i = 1:length(buildingMeshes)
        mesh = buildingMeshes(i);
        verts = mesh.Vertices;
        faces = mesh.Faces;
        avgZ = mean(verts(:,3));
        c_ratio = (avgZ-global_zmin)/(global_zmax-global_zmin+eps);
        color = baseColor + c_ratio.*(topColor-baseColor);
        patch('Vertices', verts, 'Faces', faces, ...
            'FaceColor', color, ...
            'EdgeColor', [0.75 0.8 1.0], ... % Light blue
            'FaceAlpha', 0.92, ...
            'LineWidth', 0.12);
    end
end

%% ==== 2. Charging Stations (blue squares + black edges) ====
if evalin('base','exist(''charger_pos'',''var'')')
    charger_pos = evalin('base','charger_pos');
end
if exist('charger_pos','var') && ~isempty(charger_pos)
    scatter3(charger_pos(:,1), charger_pos(:,2), charger_pos(:,3), ...
        44, 's', 'MarkerFaceColor', [0.1 0.63 0.9], ...  % Light blue
        'MarkerEdgeColor','k', 'LineWidth',1.0, ...
        'DisplayName','Charging Station');
    for i=1:size(charger_pos,1)
        text(charger_pos(i,1), charger_pos(i,2), charger_pos(i,3)+6, ...
            sprintf('C%d',i), 'Color',[0 0.39 0.8], 'FontSize',9, 'FontWeight','bold');
    end
end

%% ==== 3. Task Points (red triangles + black edges) ====
if evalin('base','exist(''tasks_3d'',''var'')')
    tasks_3d = evalin('base','tasks_3d');
end
if exist('tasks_3d','var') && ~isempty(tasks_3d)
    scatter3(tasks_3d(:,1), tasks_3d(:,2), tasks_3d(:,3), ...
        34, '^', 'MarkerFaceColor', [1 0.2 0.2], ...
        'MarkerEdgeColor','k', 'LineWidth',1.0, ...
        'DisplayName','Task Point');
    for i=1:size(tasks_3d,1)
        text(tasks_3d(i,1), tasks_3d(i,2), tasks_3d(i,3)+6, ...
            sprintf('T%d',i), 'Color',[0.8 0 0], 'FontSize',9, 'FontWeight','bold');
    end
end

%% 3. Animation preparation
% Initialize trajectory line objects and current position scatter for each UAV
trajlines = gobjects(1, n_uav);
curr_scat = gobjects(1, n_uav);
for k = 1:n_uav
    trajlines(k) = plot3(NaN,NaN,NaN,'-','Color',colors(k,:),'LineWidth',2);
    curr_scat(k) = scatter3(NaN,NaN,NaN,70,colors(k,:), 'filled', 'MarkerEdgeColor','k');
end
legendstr = arrayfun(@(x)sprintf('UAV%02d',x), uav_list,'UniformOutput',false);

%% 4. Calculate total animation duration
Tmax = max(cellfun(@(t) max(t), t_all));
dt = 0.1;  % Animation frame interval (seconds)

%% 5. Main animation loop
for t = 0:dt:Tmax
    for k = 1:n_uav
        t_curr = t_all{k};
        p_curr = pos_all{k};
        % Find indices of all trajectory points with time <= current time
        idx = find(t_curr <= t, 1, 'last');
        if ~isempty(idx)
            set(trajlines(k), ...
                'XData', p_curr(1:idx,1), ...
                'YData', p_curr(1:idx,2), ...
                'ZData', p_curr(1:idx,3));
            % Draw current head position
            set(curr_scat(k), ...
                'XData', p_curr(idx,1), ...
                'YData', p_curr(idx,2), ...
                'ZData', p_curr(idx,3));
        else
            set(trajlines(k), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            set(curr_scat(k), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        end
    end
    drawnow;
    pause(0.01); % Control animation speed, smaller value = faster
end

%% 6. Final endpoints and legend
for k = 1:n_uav
    % Draw final endpoints (diamonds)
    p_curr = pos_all{k};
    scatter3(p_curr(end,1),p_curr(end,2),p_curr(end,3),80,colors(k,:), 'd', 'filled', 'MarkerEdgeColor','k');
end

% Hide scatter for legend: "Charger Station" and "Task Point"
h_charger = scatter3(NaN,NaN,NaN,44,'s','MarkerFaceColor',[0.1 0.63 0.9],'MarkerEdgeColor','k','LineWidth',1.0);
h_task    = scatter3(NaN,NaN,NaN,34,'^','MarkerFaceColor',[1 0.2 0.2],'MarkerEdgeColor','k','LineWidth',1.0);

% Generate UAV legend names in English
uav_legend_entries = arrayfun(@(x)sprintf('UAV%02d',x), uav_list, 'UniformOutput', false);

% Combine total legend (charging stations, task points, then UAVs)
legend([h_charger, h_task, trajlines], ...
    [{'Charger Station','Task Point'}, uav_legend_entries{:}], ...
    'Location', 'Best');
hold off