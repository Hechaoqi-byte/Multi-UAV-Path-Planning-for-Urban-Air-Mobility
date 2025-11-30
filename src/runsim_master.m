function runsim_master(matfile, margin_after, run_now)
% runsim_master - convenience wrapper to run runsim with refs_for_simulink.mat
% Usage:
%  runsim_master(); % default 'refs_for_simulink.mat'
%  runsim_master('refs_for_simulink.mat', 5, true)
if nargin < 1 || isempty(matfile), matfile = 'refs_for_simulink.mat'; end
if nargin < 2 || isempty(margin_after), margin_after = 5; end
if nargin < 3, run_now = true; end

tmp = load(matfile);
if ~isfield(tmp,'refTrajectories')
    error('No refTrajectories in %s', matfile);
end
refs = tmp.refTrajectories;
if isfield(tmp,'params')
    params = tmp.params;
else
    params = crazyflie();
end

% put params in base so runsim/ref_traj read same
assignin('base','params',params);

% compute recommended time_tol considering only active refs
tmax = 0;
for i=1:numel(refs)
    r = refs{i};
    if isfield(r,'active') && ~r.active
        continue;
    end
    tmax = max(tmax, r.t(end));
end
recommended_time_tol = tmax + margin_after;
assignin('base','recommended_time_tol', recommended_time_tol);
fprintf('runsim_master: recommended_time_tol = %.2f s, params set in base workspace.\n', recommended_time_tol);

if run_now
    load(matfile);
    % runsim may read recommended_time_tol from base if coded to; otherwise edit runsim to use it.
    runsim;
end
end