function [C, b, C3, b3] = computeConstraint(order, m, k_r, k_psi, t, ...
    keyframe, corridor_position, n_intermediate, corridor_width, ...
    constraintData_r, constraintData_psi)
% computeConstraint Compute constraint matrices for Minimum Snap trajectory optimization
%
% Description:
%   Constructs the complete set of equality and inequality constraints for 
%   Minimum Snap trajectory optimization problems. This includes waypoint 
%   constraints, derivative continuity constraints, and corridor constraints
%   to ensure feasible and smooth trajectories.
%
% Input Parameters:
%   order            - Polynomial order of trajectory segments
%   m                - Number of trajectory segments
%   k_r              - Derivative order for position constraints
%   k_psi            - Derivative order for yaw angle constraints
%   t                - Time vector for segment boundaries [t0, t1, ..., t_m]
%   keyframe         - Keyframe positions [4×(m+1)] for x,y,z,psi at waypoints
%   corridor_position - Segment indices for corridor constraint application
%   n_intermediate   - Number of intermediate points for corridor constraints
%   corridor_width   - Width of the corridor constraint
%   constraintData_r - Position derivative constraints [m×k_r×3]
%   constraintData_psi - Yaw angle derivative constraints
%
% Output Parameters:
%   C                - Complete equality constraint matrix
%   b                - Complete equality constraint vector
%   C3               - Inequality constraint matrix for corridor constraints
%   b3               - Inequality constraint vector for corridor constraints
%
% Constraint Types:
%   1. Waypoint Constraints (C1, b1): Ensures trajectory passes through keyframes
%   2. Derivative Constraints (C2, b2): Ensures smoothness via velocity/acceleration continuity
%   3. Corridor Constraints (C3, b3): Keeps trajectory within specified bounds
%
% Algorithm Features:
%   - Handles multiple trajectory segments with polynomial basis functions
%   - Supports both equality and inequality constraints
%   - Provides continuity up to k_r-th derivative for smooth transitions
%   - Implements corridor constraints for obstacle avoidance and path following

n = 4;  % x,y,z,psi

%% ========= 1. Waypoint position constraints C1,b1 =========
C1 = zeros(2*m*n, n*(order+1)*m);
b1 = zeros(2*m*n, 1);
computeMat = eye(order+1);  % Polynomial basis

for i = 1:m
    if i == 1
        waypoint_start = keyframe(:, 1);
        waypoint_end   = keyframe(:, 2);
    elseif i == m
        waypoint_start = keyframe(:, m);
        waypoint_end   = keyframe(:, m+1);
    else
        waypoint_start = keyframe(:, i);
        waypoint_end   = keyframe(:, i+1);
    end

    values_start = zeros(1, order+1);
    values_end   = zeros(1, order+1);
    for j = 1:order+1
        values_start(j) = polyval(computeMat(j,:), t(i));
        values_end(j)   = polyval(computeMat(j,:), t(i+1));
    end

    % Segment i start point
    for k = 1:n
        c = zeros(1, n*(order+1)*m);
        base = (i-1)*n*(order+1) + (k-1)*(order+1);
        c(base+1:base+order+1) = values_start;
        row = (i-1)*2*n + k;
        C1(row, :) = c;
    end
    b1((i-1)*2*n + (1:n)) = waypoint_start;

    % Segment i end point
    for k = 1:n
        c = zeros(1, n*(order+1)*m);
        base = (i-1)*n*(order+1) + (k-1)*(order+1);
        c(base+1:base+order+1) = values_end;
        row = (i-1)*2*n + n + k;
        C1(row, :) = c;
    end
    b1((i-1)*2*n + n + (1:n)) = waypoint_end;
end

%% ========= 2. Derivative/continuity constraints C2,b2 =========
C2 = zeros(2*m*(n-1)*k_r, n*(order+1)*m);
b2 = zeros(2*m*(n-1)*k_r, 1);

if nargin < 10 || isempty(constraintData_r)
    % Fallback to default "all epsilon continuous" setting (extreme case)
    constraintData_r = eps * ones(m, k_r, 3);
end

computeMat = eye(order+1);

for i = 1:m
    for h = 1:k_r
        % Calculate derivative polynomial coefficients at t(i), t(i+1)
        values_ti = zeros(1, order+1);
        values_tip1 = zeros(1, order+1);
        for j = 1:order+1
            tempCoeffs = computeMat(j,:);
            for kk = 1:h
                tempCoeffs = polyder(tempCoeffs);
            end
            values_ti(j)   = polyval(tempCoeffs, t(i));
            values_tip1(j) = polyval(tempCoeffs, t(i+1));
        end

        if i == 1
            % Start point absolute derivative constraints
            for k = 1:(n-1) % x,y,z
                desired = constraintData_r(1, h, k);
                row = (h-1)*(n-1) + k;

                c = zeros(1, n*(order+1)*m);
                base = 0*n*(order+1) + (k-1)*(order+1);  % Segment 1
                c(base+1:base+order+1) = values_ti;

                C2(row, :) = c;
                if abs(desired - eps) < 1e-12
                    b2(row) = 0;
                else
                    b2(row) = desired;
                end
            end
        else
            % Other segments: at connection point t(i)
            % 2.1 Continuity
            for k = 1:(n-1)
                isContinuity = (constraintData_r(i, h, k) == eps);
                if isContinuity
                    row = (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r + k;

                    c = zeros(1, n*(order+1)*m);
                    base_prev = (i-2)*n*(order+1) + (k-1)*(order+1);
                    base_curr = (i-1)*n*(order+1) + (k-1)*(order+1);
                    c(base_prev+1:base_prev+order+1) = values_ti;
                    c(base_curr+1:base_curr+order+1) = -values_ti;

                    C2(row, :) = c;
                    b2(row) = 0;
                end
            end

            % 2.2 If not continuity, then absolute derivative constraint
            for k = 1:(n-1)
                desired = constraintData_r(i, h, k);
                if abs(desired - eps) > 1e-12
                    row = (h-1)*(n-1) + 2*(i-1)*(n-1)*k_r + (n-1)*k_r + k;

                    c = zeros(1, n*(order+1)*m);
                    base_curr = (i-1)*n*(order+1) + (k-1)*(order+1);
                    c(base_curr+1:base_curr+order+1) = values_ti;

                    C2(row, :) = c;
                    b2(row) = desired;
                end
            end
        end
    end
end

%% ========= 3. Corridor constraints C3,b3 (basically unchanged) =========
C3 = [];
b3 = [];

if nargin >= 7 && ~isempty(corridor_position) && ...
        ~isempty(n_intermediate) && ~isempty(corridor_width)

    t_vector = (keyframe(1:3,corridor_position(2)) - keyframe(1:3,corridor_position(1))) ...
        / norm(keyframe(1:3,corridor_position(2)) - keyframe(1:3,corridor_position(1)));

    t_intermediate = linspace(t(corridor_position(1)), t(corridor_position(2)), n_intermediate+2);
    t_intermediate = t_intermediate(2:end-1);

    computeMat = eye(order+1);
    for i = 1:n_intermediate
        values = zeros(1, order+1);
        for j = 1:order+1
            values(j) = polyval(computeMat(j,:), t_intermediate(i));
        end

        c = zeros(6, n*(order+1)*m);
        b = zeros(6, 1);

        rix = keyframe(1, corridor_position(1));
        riy = keyframe(2, corridor_position(1));
        riz = keyframe(3, corridor_position(1));

        % Original corridor inequality constraints (omitted, keep original logic)
        c(1,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = [values zeros(1,2*(order+1))] ...
            - t_vector(1)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
        b(1) = corridor_width + ...
            rix+t_vector(1)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        c(2,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = -c(1,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
        b(2) = corridor_width + ...
            -rix-t_vector(1)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        c(3,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = [zeros(1,order+1) values zeros(1,order+1)] ...
            - t_vector(2)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
        b(3) = corridor_width + ...
            riy+t_vector(2)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        c(4,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = -c(3,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
        b(4) = corridor_width + ...
            -riy-t_vector(2)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        c(5,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = [zeros(1,2*(order+1)) values] ...
            - t_vector(3)*[t_vector(1)*values t_vector(2)*values t_vector(3)*values];
        b(5) = corridor_width + ...
            riz+t_vector(3)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        c(6,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1)) ...
            = -c(5,(corridor_position(1)-1)*n*(order+1)+1:(corridor_position(1)-1)*n*(order+1)+3*(order+1));
        b(6) = corridor_width + ...
            -riz-t_vector(3)*(-rix*t_vector(1) -riy*t_vector(2) -riz*t_vector(3));

        C3 = [C3; c];
        b3 = [b3; b];
    end
end

%% ========= 4. Summary =========
C = [C1; C2];
b = [b1; b2];
end