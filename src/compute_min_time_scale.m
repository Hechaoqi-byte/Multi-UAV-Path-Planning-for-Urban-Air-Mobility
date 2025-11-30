function res = compute_min_time_scale(ref, params, varargin)
% Robust compute_min_time_scale that tolerates various ref.acc shapes.
% Keeps same API as before.
opts = struct('rotor_count',4, 's_max',20, 'tol',1e-3);
for k=1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k+1};
end

m = params.mass;
g = params.grav;
maxF = params.maxF;
maxAngle = params.maxangle;
rotors = opts.rotor_count;

% normalize acc to Nx3
acc = [];
if isfield(ref,'acc')
    acc = ref.acc;
end
if isempty(acc)
    % nothing to check; treat as trivial (no accel => feasible)
    res.maxFreq = 0;
    res.maxFper = 0;
    res.viol_times_F = [];
    res.viol_times_angle = [];
    res.s_est = 1;
    res.s_min = 1;
    return;
end

% acc may be (N x 3) or (3 x N) or (1 x 3) etc. Convert to N x 3.
sz = size(acc);
if sz(2) == 3 && sz(1) >= 1
    accN = acc;            % N x 3
elseif sz(1) == 3 && sz(2) >= 1
    accN = acc';           % transpose to N x 3
elseif numel(acc) == 3
    accN = reshape(acc, 1, 3);
else
    % unknown shape: try to linearize if possible
    try
        accv = acc(:);
        if mod(numel(accv),3) == 0
            accN = reshape(accv, 3, [])';
        else
            % cannot reshape to Nx3
            warning('compute_min_time_scale:badacc', 'ref.acc has unexpected size [%d x %d] and cannot be reshaped to Nx3', sz(1), sz(2));
            res.maxFreq = NaN; res.maxFper = NaN; res.viol_times_F = []; res.viol_times_angle = [];
            res.s_est = NaN; res.s_min = NaN;
            return;
        end
    catch
        warning('compute_min_time_scale:badacc2', 'ref.acc cannot be parsed; returning NaN');
        res.maxFreq = NaN; res.maxFper = NaN; res.viol_times_F = []; res.viol_times_angle = [];
        res.s_est = NaN; res.s_min = NaN;
        return;
    end
end

N = size(accN,1);
a_des = accN';   % 3 x N

% compute original (s=1) values
Freq = zeros(N,1);
tilt = zeros(N,1);
for i=1:N
    a_i = a_des(:,i);
    a_with_g = a_i + [0;0;g];
    Freq(i) = m * norm(a_with_g);
    % guard against zero vector
    if norm(a_with_g) == 0
        b3z = 1;
    else
        b3 = a_with_g / norm(a_with_g);
        b3z = b3(3);
    end
    tilt(i) = acos( min(max(b3z,-1),1) );
end
F_per = Freq / rotors;

res.maxFreq = max(Freq);
res.maxFper = max(F_per);
res.viol_times_F = []; res.viol_times_angle = [];
tvec = ref.t(:);
if numel(tvec) == N
    res.viol_times_F = tvec(F_per > maxF);
    res.viol_times_angle = tvec(tilt > maxAngle);
else
    % if time vector length mismatch, just return indices
    res.viol_times_F = find(F_per > maxF);
    res.viol_times_angle = find(tilt > maxAngle);
end

% simple thrust-based estimate
if res.maxFreq > 0 && ~isnan(res.maxFreq)
    res.s_est = max(1, sqrt( res.maxFreq / (rotors * maxF) ));
else
    res.s_est = 1;
end

% If already feasible
if all(F_per <= maxF) && all(tilt <= maxAngle)
    res.s_min = 1;
    return;
end

% find feasible hi bound
lo = 1; hi = max(2, res.s_est);
ok_hi = false;
while hi <= opts.s_max
    feasible = true;
    for i=1:N
        a_with_g = (a_des(:,i)/(hi^2)) + [0;0;g];
        Freq_s = m * norm(a_with_g);
        if Freq_s / rotors > maxF
            feasible = false; break;
        end
        if norm(a_with_g) == 0
            b3z = 1;
        else
            b3 = a_with_g / norm(a_with_g);
            b3z = b3(3);
        end
        if acos(min(max(b3z,-1),1)) > maxAngle
            feasible = false; break;
        end
    end
    if feasible, ok_hi = true; break; end
    hi = hi * 2;
end

if ~ok_hi
    res.s_min = NaN;
    return;
end

% bisection
while (hi - lo) > opts.tol
    mid = (lo + hi) / 2;
    feasible = true;
    for i=1:N
        a_with_g = (a_des(:,i)/(mid^2)) + [0;0;g];
        Freq_s = m * norm(a_with_g);
        if Freq_s / rotors > maxF
            feasible = false; break;
        end
        if norm(a_with_g) == 0
            b3z = 1;
        else
            b3 = a_with_g / norm(a_with_g);
            b3z = b3(3);
        end
        if acos(min(max(b3z,-1),1)) > maxAngle
            feasible = false; break;
        end
    end
    if feasible
        hi = mid;
    else
        lo = mid;
    end
end
res.s_min = hi;
end