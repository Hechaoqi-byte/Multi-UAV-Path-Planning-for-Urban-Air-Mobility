function [desired_state] = ref_traj(t, qn)
% ref_traj: trajectory handle for runsim that returns desired_state using refTrajectories
% Requires refTrajectories to be in base workspace (or refs_for_simulink.mat in cwd)

persistent refs_loaded refTrajectories_local

if isempty(refs_loaded)
    if evalin('base','exist(''refTrajectories'',''var'')')
        refTrajectories_local = evalin('base','refTrajectories');
    elseif exist('refs_for_simulink.mat','file')
        tmp = load('refs_for_simulink.mat','refTrajectories');
        if isfield(tmp,'refTrajectories')
            refTrajectories_local = tmp.refTrajectories;
            assignin('base','refTrajectories', refTrajectories_local);
        else
            error('refs_for_simulink.mat exists but refTrajectories not found in it.');
        end
    else
        error('refTrajectories not found. Run exportRefsForSimulink and load refs_for_simulink.mat into workspace.');
    end
    refs_loaded = true;
end

% safety checks
if qn > numel(refTrajectories_local) || isempty(refTrajectories_local{qn})
    error('ref_traj: no ref for quad %d', qn);
end
ref = refTrajectories_local{qn};

tvec = ref.t(:);

if t <= tvec(1)
    pos = ref.pos(1,:)';
    vel = ref.vel(1,:)';
    acc = ref.acc(1,:)';
    yaw = ref.yaw(1);
    yawdot = ref.yawdot(1);
elseif t >= tvec(end)
    pos = ref.pos(end,:)';
    vel = zeros(3,1);
    acc = zeros(3,1);
    yaw = ref.yaw(end);
    yawdot = 0;
else
    pos = interp1(tvec, ref.pos, t, 'pchip')';
    vel = interp1(tvec, ref.vel, t, 'pchip')';
    acc = interp1(tvec, ref.acc, t, 'pchip')';
    yaw = interp1(tvec, ref.yaw, t, 'pchip');
    yawdot = interp1(tvec, ref.yawdot, t, 'pchip');
end

desired_state.pos = pos;
desired_state.vel = vel;
desired_state.acc = acc;
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
end