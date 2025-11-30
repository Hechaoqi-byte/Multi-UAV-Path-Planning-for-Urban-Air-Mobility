function status = export_and_run_active_uavs(varargin)
% export_and_run_active_uavs  Export only active optimizedRefPaths and run runsim.
% Default behavior now: sequential single-UAV runs with aggressive safety margin.
% Usage:
%   status = export_and_run_active_uavs(); % sequential by default
%   status = export_and_run_active_uavs('mode','parallel'); % override

% Parse options
p = inputParser;
addParameter(p,'savefile','refs_for_simulink.mat',@ischar);
addParameter(p,'samplesPerUAV',400,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'totalTimePerUAV',60,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'auto_scale',true,@islogical);         % use compute_min_time_scale
addParameter(p,'global_scale',false,@islogical);      % default: per-UAV scaling
addParameter(p,'run_sim',true,@islogical);
addParameter(p,'safety_margin',2.0,@(x)isnumeric(x)&&x>0); % larger margin by default
addParameter(p,'verbose',true,@islogical);
addParameter(p,'mode','sequential',@(s)ischar(s) && any(strcmpi(s,{'parallel','sequential'}))); % default sequential
addParameter(p,'sequential_pause',1,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'close_figs_after_run',true,@islogical);
parse(p,varargin{:});
opts = p.Results;

status.success = false;
status.phase = '';
status.msg = '';

% Get optimizedRefPaths from base workspace
if evalin('base','exist(''optimizedRefPaths'',''var'')')==1
    optimizedRefPaths = evalin('base','optimizedRefPaths');
else
    error('optimizedRefPaths not found in base workspace. Run planning first.');
end
if ~iscell(optimizedRefPaths)
    error('optimizedRefPaths must be a cell array.');
end

% Find active UAVs (non-empty, >1 point)
activeIdx = find(cellfun(@(c) ~isempty(c) && isnumeric(c) && size(c,1)>1, optimizedRefPaths));
if isempty(activeIdx)
    status.msg = 'No active optimizedRefPaths found.';
    if opts.verbose, fprintf('%s\n',status.msg); end
    return;
end
if opts.verbose, fprintf('Active UAVs: %s\n', mat2str(activeIdx)); end

% Helper: create robust 13x1 init state from a ref struct
% Uses init_state(start,yaw) if available; otherwise builds one and inserts ref initial vel & yaw.
    function xi = get_init_state_from_ref(ref)
        % ref: struct with fields pos (Nx3), optionally vel (Nx3), yaw (Nx1)
        % returns 13x1 column vector: [pos(3); vel(3); quat(4) qw,qx,qy,qz; ang(3)]
        % prefer existing helper
        pos0 = [0;0;0];
        if isfield(ref,'pos') && ~isempty(ref.pos)
            pos0 = ref.pos(1,:)';
        end
        vel0 = [0;0;0];
        if isfield(ref,'vel') && ~isempty(ref.vel)
            vel0 = ref.vel(1,:)';
        end
        yaw0 = 0;
        if isfield(ref,'yaw') && ~isempty(ref.yaw)
            yaw0 = ref.yaw(1);
        end

        % If project has init_state function, use it to keep consistent quaternion convention
        if exist('init_state','file')==2
            try
                xi = init_state(pos0, yaw0); % sets zero velocity by default
                % overwrite velocities if reference has them
                xi(4:6) = vel0;
                return;
            catch
                % fall through to manual construction
            end
        end

        % Manual construction: quaternion from yaw (roll=0, pitch=0)
        qw = cos(yaw0/2);
        qz = sin(yaw0/2);
        quat0 = [qw; 0; 0; qz]; % [qw qx qy qz]
        xi = zeros(13,1);
        xi(1:3) = pos0;
        xi(4:6) = vel0;
        xi(7:10) = quat0;
        xi(11:13) = [0;0;0];
    end

% Helper: export & run single set of activeRefs (cell array)
    function st = do_export_and_run(singleActiveRefs, singleActiveIdx, savefile_local)
        st.success = false; st.msg = '';
        try
            if opts.auto_scale && exist('exportRefsForSimulink','file')==2
                % let that function create refs; we'll try to overwrite init_states with better guesses
                exportRefsForSimulink(singleActiveRefs, savefile_local, opts.totalTimePerUAV, opts.samplesPerUAV, ...
                    'auto_scale', opts.auto_scale, 'global_scale', opts.global_scale);
                % attempt to load generated refs and update init_states using ref initial vel/yaw
                try
                    tmp = load(savefile_local,'refTrajectories','init_states');
                    if isfield(tmp,'refTrajectories')
                        refTrajectories_local = tmp.refTrajectories;
                        init_states_local = cell(numel(refTrajectories_local),1);
                        for kk = 1:numel(refTrajectories_local)
                            init_states_local{kk} = get_init_state_from_ref(refTrajectories_local{kk});
                        end
                        % overwrite matfile init_states with improved init_states_local
                        refTrajectories = refTrajectories_local; init_states = init_states_local; 
                        save(savefile_local,'refTrajectories','init_states','-append');
                    end
                catch
                    % ignore and proceed if something goes wrong here
                end
            else
                % fallback export using buildRefTrajectory (if available)
                refTrajectories_local = cell(numel(singleActiveRefs),1);
                init_states_local = cell(numel(singleActiveRefs),1);
                for kk = 1:numel(singleActiveRefs)
                    path = singleActiveRefs{kk}(:,1:3);
                    samples = min(opts.samplesPerUAV, size(path,1));
                    T = opts.totalTimePerUAV;
                    if exist('buildRefTrajectory','file')==2
                        ref = buildRefTrajectory(path, T, samples);
                        refTrajectories_local{kk} = struct('t', ref.t, 'pos', ref.pos, 'vel', ref.vel, 'acc', ref.acc, ...
                            'yaw', ref.yaw, 'yawdot', ref.yawdot, 'active', true, 'orig_index', singleActiveIdx(kk));
                        % use reference initial velocity & yaw for init state
                        init_states_local{kk} = get_init_state_from_ref(ref);
                    else
                        tvec = linspace(0,T,samples)';
                        refTrajectories_local{kk} = struct('t', tvec', 'pos', path, 'vel', zeros(samples,3), 'acc', zeros(samples,3), ...
                            'yaw', zeros(samples,1), 'yawdot', zeros(samples,1), 'active', true, 'orig_index', singleActiveIdx(kk));
                        % build a temporary ref-like struct to extract init
                        reftmp.t = tvec';
                        reftmp.pos = path;
                        reftmp.vel = zeros(samples,3);
                        reftmp.yaw = zeros(samples,1);
                        init_states_local{kk} = get_init_state_from_ref(reftmp);
                    end
                end
                % save using temp names then normalize names for runsim
                save(savefile_local,'refTrajectories_local','init_states_local','-v7.3');
                refTrajectories = refTrajectories_local; init_states = init_states_local; 
                save(savefile_local,'refTrajectories','init_states','-append');
            end
            % load and assign to base (kept for compatibility; runsim_master will load the matfile too)
            tmpLocal = load(savefile_local,'refTrajectories','init_states');
            if isfield(tmpLocal,'refTrajectories'), assignin('base','refTrajectories', tmpLocal.refTrajectories); end
            if isfield(tmpLocal,'init_states'), assignin('base','init_states', tmpLocal.init_states); end
            assignin('base','uav_map_active2orig', singleActiveIdx);
            if opts.run_sim
                runsim_master(savefile_local, 5, true);
            end
            st.success = true;
        catch ME
            st.success = false;
            st.msg = ME.message;
        end
    end

% Mode handling
switch lower(opts.mode)
    case 'parallel'
        % compress all active refs and run once
        activeRefs = cell(1,numel(activeIdx));
        for k=1:numel(activeIdx)
            activeRefs{k} = optimizedRefPaths{activeIdx(k)};
        end
        st = do_export_and_run(activeRefs, activeIdx, opts.savefile);
        if ~st.success
            status.phase = 'failed';
            status.msg = st.msg;
            if opts.verbose, fprintf('Parallel run failed: %s\n', st.msg); end
            return;
        end
        status.success = true; status.phase = 'ran'; status.msg = 'Parallel run completed';
        if opts.verbose, fprintf('%s\n', status.msg); end

    case 'sequential'
        % iterate each active UAV and run individually
        for k = 1:numel(activeIdx)
            u = activeIdx(k);
            if opts.verbose, fprintf('Sequential run: UAV %d (%d/%d)\n', u, k, numel(activeIdx)); end
            singleRef = { optimizedRefPaths{u} };
            singleIdx = u;
            % IMPORTANT: save to the fixed filename that runsim expects (overwrite each time)
            savefile_single = opts.savefile; % overwrite refs_for_simulink.mat so runsim loads correct file
            if opts.verbose, fprintf('Writing single-UAV refs to %s (overwriting)...\n', savefile_single); end
            st = do_export_and_run(singleRef, singleIdx, savefile_single);
            if ~st.success
                % try automatic slow-down retry for this UAV only
                if opts.verbose, warning('Run failed for UAV %d: %s\nTrying slow-down retry...', u, st.msg); end
                try
                    % compute s_min if possible
                    if exist('buildRefTrajectory','file')==2 && exist('compute_min_time_scale','file')==2
                        ref_tmp = buildRefTrajectory(optimizedRefPaths{u}(:,1:3), opts.totalTimePerUAV, opts.samplesPerUAV);
                        params = crazyflie();
                        res = compute_min_time_scale(ref_tmp, params, 'rotor_count', 4, 's_max', 50, 'tol', 1e-3);
                        s_val = res.s_min; if isnan(s_val), s_val = res.s_est; end
                        s_use = max(1, s_val) * opts.safety_margin;
                        % regenerate scaled ref and run
                        t_old = ref_tmp.t(:); samples = size(ref_tmp.pos,1);
                        t_new = linspace(0, t_old(end)*s_use, samples)';
                        t_arg = t_new / s_use;
                        pos_new = interp1(t_old, ref_tmp.pos, t_arg, 'pchip');
                        vel_new = interp1(t_old, ref_tmp.vel, t_arg, 'pchip') / s_use;
                        acc_new = interp1(t_old, ref_tmp.acc, t_arg, 'pchip') / (s_use^2);
                        refScaled = struct('t', t_new', 'pos', pos_new, 'vel', vel_new, 'acc', acc_new, ...
                                           'yaw', interp1(t_old,ref_tmp.yaw,t_arg,'pchip'), ...
                                           'yawdot', interp1(t_old,ref_tmp.yawdot,t_arg,'pchip')/s_use);
                        refTrajectories = { struct('t',refScaled.t,'pos',refScaled.pos,'vel',refScaled.vel,'acc',refScaled.acc,'yaw',refScaled.yaw,'yawdot',refScaled.yawdot,'active',true,'orig_index',u) };
                        % use init state based on reference initial vel/yaw
                        init_states = { get_init_state_from_ref(refScaled) };
                        save(savefile_single,'refTrajectories','init_states','-v7.3');
                        assignin('base','refTrajectories', refTrajectories);
                        assignin('base','init_states', init_states);
                        assignin('base','uav_map_active2orig', u);
                        runsim_master(savefile_single,5,true);
                        if opts.verbose, fprintf('Retry succeeded for UAV %d with s=%.3g\n', u, s_use); end
                    else
                        if opts.verbose, warning('Cannot compute s_min (missing buildRefTrajectory/compute_min_time_scale).'); end
                    end
                catch ME2
                    if opts.verbose, fprintf('Retry also failed for UAV %d: %s\n', u, ME2.message); end
                end
            end
            % optionally close figures to avoid overlaying plots
            if opts.close_figs_after_run
                try close all; catch, end
            end
            % pause between sequential runs
            pause(max(0, opts.sequential_pause));
        end
        status.success = true; status.phase = 'ran_sequential'; status.msg = 'Sequential runs completed';
        if opts.verbose, fprintf('%s\n', status.msg); end

    otherwise
        error('Unknown mode: %s', opts.mode);
end

end