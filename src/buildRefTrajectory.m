function ref = buildRefTrajectory(path_xyz, T_total, samples)
% buildRefTrajectory - build a time-indexed reference struct for Simulink
% Inputs:
%   path_xyz : [N x 3] waypoints
%   T_total  : total duration to traverse path (seconds)
%   samples  : number of samples (e.g. 200)
% Output:
%   ref struct with fields: t (1xM), pos (Mx3), vel (Mx3), acc (Mx3), yaw (Mx1), yawdot (Mx1)

if nargin < 3, samples = 200; end
if nargin < 2 || isempty(T_total), T_total = max(20, size(path_xyz,1)*0.5); end

% remove duplicates
if size(path_xyz,1) > 1
    diffs = vecnorm(diff(path_xyz,1,1),2,2);
    keep = [true; diffs > 1e-6];
    path_xyz = path_xyz(keep,:);
end

% arc-length parameterization
seg = [0; vecnorm(diff(path_xyz,1,1),2,2)];
s = cumsum(seg);
sMax = s(end);
if sMax < 1e-6
    % degenerate: duplicate a point
    path_xyz = repmat(path_xyz(1,:),2,1);
    seg = [0; vecnorm(diff(path_xyz,1,1),2,2)];
    s = cumsum(seg);
    sMax = s(end);
end

% uniform samples in arc-length
s_query = linspace(0, sMax, samples)';
t_query = linspace(0, T_total, samples)';

% position interpolation by arc-length
pos = interp1(s, path_xyz, s_query, 'pchip');

% approximate velocity/acceleration by finite differences over time
dt = T_total/(samples-1);
vel = [diff(pos)/dt; zeros(1,3)];
acc = [diff(vel)/dt; zeros(1,3)];

% yaw: compute tangent heading in XY
tangents = zeros(samples,3);
for i=1:samples
    if i==1, v = pos(2,:) - pos(1,:);
    elseif i==samples, v = pos(end,:) - pos(end-1,:);
    else v = pos(i+1,:) - pos(i-1,:);
    end
    n = norm(v);
    if n < 1e-6, tangents(i,:) = [1 0 0]; else tangents(i,:) = v/n; end
end
yaw = atan2(tangents(:,2), tangents(:,1));
yawdot = [diff(yaw)/dt; 0];

ref.t = t_query';
ref.pos = pos;
ref.vel = vel;
ref.acc = acc;
ref.yaw = yaw;
ref.yawdot = yawdot;
end