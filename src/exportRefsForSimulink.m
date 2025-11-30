function exportRefsForSimulink(optimizedRefPaths, savefile, totalTimePerUAV, samplesPerUAV, varargin)
% exportRefsForSimulink - convert planner output (optimizedRefPaths) to
% refTrajectories / init_states for runsim/Simulink and save to MAT.
%
% Usage:
%  exportRefsForSimulink(optimizedRefPaths, 'refs_for_simulink.mat', 60, 400)
%  exportRefsForSimulink(..., totalTimePerUAV, samplesPerUAV, 'auto_scale', true, 'maxF_factor', 1.2)
%
% Options:
%  'auto_scale' (default true) : compute minimal time scaling s per ref using params
%  'global_scale' (default true): apply same s to all active UAVs
%  's_max' (default 20) : max s to search
%  'maxF_factor' (default 1.0) : temporarily relax single-motor maxF by this factor
%  'maxangle_deg' (default []) : temporarily set maxangle (deg) if provided

if nargin < 2 || isempty(savefile), savefile = 'refs_for_simulink.mat'; end
if nargin < 3 || isempty(totalTimePerUAV), totalTimePerUAV = []; end
if nargin < 4 || isempty(samplesPerUAV), samplesPerUAV = 400; end

opts = struct('auto_scale', true, 'global_scale', true, 's_max', 20, 'maxF_factor', 1.0, 'maxangle_deg', []);
for k=1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k+1};
end

numUAVs = numel(optimizedRefPaths);
raw_refs = cell(numUAVs,1);

% Build raw refs (or mark empty)
for u = 1:numUAVs
    path = optimizedRefPaths{u};
    if isempty(path) || size(path,1) < 2
        raw_refs{u} = []; % inactive / placeholder
        continue;
    end
    % default total time (if not provided) - use 60s as placeholder; will scale later
    if isempty(totalTimePerUAV)
        T = 60;
    else
        if numel(totalTimePerUAV) == 1, T = totalTimePerUAV; else T = totalTimePerUAV(u); end
    end
    ref = buildRefTrajectory(path, T, samplesPerUAV);
    raw_refs{u} = ref;
end

% Load params and possibly relax limits
params = crazyflie();
params.maxF = params.maxF * opts.maxF_factor;
if ~isempty(opts.maxangle_deg)
    params.maxangle = deg2rad(opts.maxangle_deg);
end

% auto-scaling: compute s_min for active refs
s_vec = ones(numUAVs,1);
active = false(numUAVs,1);
if opts.auto_scale
    s_min_list = nan(numUAVs,1);
    for u=1:numUAVs
        ref = raw_refs{u};
        if isempty(ref)
            active(u) = false;
            continue;
        end
        active(u) = true;
        % compute s_min using helper
        res = compute_min_time_scale(ref, params, 'rotor_count', 4, 's_max', opts.s_max, 'tol', 1e-3);
        s_min_list(u) = res.s_min;
        % if returned NaN, fallback to s_est
        if isnan(s_min_list(u))
            s_min_list(u) = res.s_est;
        end
    end
    if any(active)
        if opts.global_scale
            s_use = ceil(max(s_min_list(active))*10)/10; % conservative rounding
            s_vec(active) = s_use;
        else
            s_vec(active) = ceil(s_min_list(active)*10)/10;
            s_vec(~active) = 1;
        end
    end
else
    % if totalTimePerUAV input provided, use it; otherwise default 60
    if ~isempty(totalTimePerUAV)
        if numel(totalTimePerUAV)==1
            % handle empty raw_refs entries safely
            t_ends = cellfun(@(r) (isempty(r) && 1) || r.t(end), raw_refs);
            s_vec(:) = totalTimePerUAV ./ t_ends;
        else
            for u=1:numUAVs
                if isempty(raw_refs{u})
                    s_vec(u) = 1;
                else
                    s_vec(u) = totalTimePerUAV(u) / raw_refs{u}.t(end);
                end
            end
        end
    else
        s_vec(:) = 1;
    end
end

% Build final refs and init_states; mark active flag per ref
refTrajectories = cell(0,1);
init_states = cell(0,1);
% Keep original index mapping so user can track indices if needed
orig_idx = zeros(numUAVs,1);
out_idx = 0;
for u = 1:numUAVs
    ref = raw_refs{u};
    if isempty(ref)
        % keep inactive placeholder: small hover at origin to save storage (but mark inactive)
        out_idx = out_idx + 1;
        refTrajectories{out_idx} = struct('t', 0, 'pos', zeros(1,3), 'vel', zeros(1,3), 'acc', zeros(1,3), 'yaw', 0, 'yawdot', 0, 'active', false, 'orig_index', u);
        init_states{out_idx} = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0];
        orig_idx(u) = out_idx;
        continue;
    end
    s = s_vec(u);
    samples = size(ref.pos,1);
    t_old = ref.t(:);
    % Correct time-stretching: compute t_new first, then compute t_arg = t_new / s
    t_new = linspace(0, t_old(end)*s, samples)';
    t_arg = t_new / s;
    pos_new = interp1(t_old, ref.pos, t_arg, 'pchip');
    vel_new = interp1(t_old, ref.vel, t_arg, 'pchip') / s;
    acc_new = interp1(t_old, ref.acc, t_arg, 'pchip') / (s^2);
    yaw_new = interp1(t_old, ref.yaw, t_arg, 'pchip');
    ydot_new = interp1(t_old, ref.yawdot, t_arg, 'pchip') / s;
    out_idx = out_idx + 1;
    refTrajectories{out_idx} = struct('t', t_new', 'pos', pos_new, 'vel', vel_new, 'acc', acc_new, 'yaw', yaw_new, 'yawdot', ydot_new, 'active', true, 'orig_index', u, 'scale', s);
    init_states{out_idx} = [pos_new(1,:)'; 0;0;0; 1;0;0;0; 0;0;0];
    orig_idx(u) = out_idx;
end

% Save params and refs together
save(savefile,'refTrajectories','init_states','params','-v7.3');
fprintf('Saved %d refs to %s (original slots=%d). Active=%d. Global scale applied? %d\n', numel(refTrajectories), savefile, numUAVs, sum(cellfun(@(r) r.active, refTrajectories)), opts.global_scale);
end