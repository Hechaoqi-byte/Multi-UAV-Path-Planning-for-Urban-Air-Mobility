% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** MEAM 620 QUADROTOR SIMULATION *****************
close all
clear all
  % 或 load('city_mesh.mat');  文件名以你文件实际为准
% === load exported refs (if present) ===
if exist('refs_for_simulink.mat','file')
    tmp = load('refs_for_simulink.mat','refTrajectories','init_states');
    if isfield(tmp,'refTrajectories'), refTrajectories = tmp.refTrajectories; end
    if isfield(tmp,'init_states'), init_states = tmp.init_states; end
    fprintf('Loaded refTrajectories (num=%d) from refs_for_simulink.mat\n', numel(refTrajectories));
end

% You can change trajectory here


trajhandle = @ref_traj;
% controller
controlhandle = @controller;

% real-time 
real_time = false;

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********
% number of quadrotors
% number of quadrotors (auto from exported refs if available)
if exist('refTrajectories','var')
    nquad = numel(refTrajectories);
else
    nquad = 1;
end
% max time
time_tol =1000;

% parameters for simulation
params = crazyflie();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')
hold on
%% ==== 1. 建筑物 Mesh（浅色背景） ====
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

    % ==== 地面区域边界 ====
    xMin = min(all_verts(:,1));
    xMax = max(all_verts(:,1));
    yMin = min(all_verts(:,2));
    yMax = max(all_verts(:,2));

    % ==== 画绿色地面 ====
    fill3([xMin xMax xMax xMin], [yMin yMin yMax yMax], [0 0 0 0], ...
        [0.82 1 0.82], ... % 浅绿色
        'FaceAlpha', 0.63, 'EdgeColor', 'none');

    % ==== 建筑物mesh ====
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
            'EdgeColor', [0.75 0.8 1.0], ... % 淡蓝
            'FaceAlpha', 0.92, ...
            'LineWidth', 0.12);
    end
end

%% ==== 2. 充电站（蓝色小方块+黑边） ====
if evalin('base','exist(''charger_pos'',''var'')')
    charger_pos = evalin('base','charger_pos');
end
if exist('charger_pos','var') && ~isempty(charger_pos)
    scatter3(charger_pos(:,1), charger_pos(:,2), charger_pos(:,3), ...
        44, 's', 'MarkerFaceColor', [0.1 0.63 0.9], ...  % 浅蓝
        'MarkerEdgeColor','k', 'LineWidth',1.0, ...
        'DisplayName','充电站');
    for i=1:size(charger_pos,1)
        text(charger_pos(i,1), charger_pos(i,2), charger_pos(i,3)+6, ...
            sprintf('C%d',i), 'Color',[0 0.39 0.8], 'FontSize',9, 'FontWeight','bold');
    end
end

%% ==== 3. 任务点（红色小三角+黑边） ====
if evalin('base','exist(''tasks_3d'',''var'')')
    tasks_3d = evalin('base','tasks_3d');
end
if exist('tasks_3d','var') && ~isempty(tasks_3d)
    scatter3(tasks_3d(:,1), tasks_3d(:,2), tasks_3d(:,3), ...
        34, '^', 'MarkerFaceColor', [1 0.2 0.2], ...
        'MarkerEdgeColor','k', 'LineWidth',1.0, ...
        'DisplayName','任务点');
    for i=1:size(tasks_3d,1)
        text(tasks_3d(i,1), tasks_3d(i,2), tasks_3d(i,3)+6, ...
            sprintf('T%d',i), 'Color',[0.8 0 0], 'FontSize',9, 'FontWeight','bold');
    end
end
lighting gouraud
camlight
hold off

% ------------------ INITIAL CONDITIONS (robust) ------------------
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.1;      % this determines the time step at which the solution is given
cstep     = 0.5;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

% Prepare containers
x0 = cell(nquad,1);
xtraj = cell(nquad,1);
ttraj = cell(nquad,1);
stop  = cell(nquad,1);

for qn = 1:nquad
    % If exported init_states exist, use them; otherwise build from traj
    if exist('init_states','var') && numel(init_states) >= qn && ~isempty(init_states{qn})
        xi = init_states{qn}(:);            % force column vector
        % If xi is too short, build fallback full-state from reference start
        if numel(xi) < 13
            if exist('refTrajectories','var') && numel(refTrajectories) >= qn
                pos0 = refTrajectories{qn}.pos(1,:)';
            else
                % last resort call trajhandle
                ds = trajhandle(0, qn);
                pos0 = ds.pos(:);
            end
            % full state: pos(3); vel(3)=0; quat(4)=[1;0;0;0]; ang(3)=0 => 13 elems
            xi = [pos0; 0;0;0; 1;0;0;0; 0;0;0];
            warning('init_states{%d} too short -> using fallback full state', qn);
        end
        x0{qn} = xi;
    else
        % No init_states provided: build full-state from trajhandle/ref
        if exist('refTrajectories','var') && numel(refTrajectories) >= qn
            pos0 = refTrajectories{qn}.pos(1,:)';
        else
            ds = trajhandle(0, qn);
            pos0 = ds.pos(:);
        end
        x0{qn} = [pos0; 0;0;0; 1;0;0;0; 0;0;0];  % 13 elements
    end

    % Ensure x0 is a column vector
    x0{qn} = x0{qn}(:);

    % Pre-allocate history arrays using the actual state length
    xtraj{qn} = zeros(max_iter*nstep, numel(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);

    % stop will be set below from refTrajectories (or trajhandle fallback)
    stop{qn} = [];
end

% Ensure stop positions are set for termination check (3x1 column vectors)
if exist('refTrajectories','var')
    for qn = 1:nquad
        stop{qn} = refTrajectories{qn}.pos(end,:)';
    end
else
    for qn = 1:nquad
        des_stop = trajhandle(inf, qn);
        stop{qn} = des_stop.pos(:);
    end
end

% Set simulation state to initial conditions
x = x0;        % state cell array
pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
OUTPUT_TO_VIDEO = 0;
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('diamond.avi');
    open(v)
end

fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter
    iter;
    timeint = time:tstep:time+cstep;

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
        if OUTPUT_TO_VIDEO == 1
            im = frame2im(getframe(gcf));
            writeVideo(v,im);
        end
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
end

if OUTPUT_TO_VIDEO == 1
    close(v);
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot
for qn = 1:nquad
    % Truncate saved variables
    QP{qn}.TruncateHist();
    % Plot position for each quad
    h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
    plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
    plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
    % Plot velocity for each quad
    h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
    plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
    plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
end
if(~isempty(err))
    error(err);
end


fprintf('finished.\n')


% 获取当前无人机编号，设为uav_id
if evalin('base','exist(''uav_map_active2orig'',''var'')')
    uav_map = evalin('base','uav_map_active2orig');
    % sequential运行就1台无人机，对应编号
    if iscell(uav_map)
        uav_id = uav_map{1};
    else
        uav_id = uav_map(1);
    end
else
    uav_id = 1; % fallback
end

% 保存这次仿真的轨迹和参考轨迹
savename = sprintf('traj_result_uav%02d.mat', uav_id);
save(savename, 'xtraj', 'ttraj', 'refTrajectories');
fprintf('轨迹数据保存在 %s\n', savename);