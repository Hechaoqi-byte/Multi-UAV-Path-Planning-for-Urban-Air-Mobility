function trajectory = extractTrajectoryFromSolution_new(solution, order, m, t)
    % Extract trajectory from QP solution (following reference implementation format)
    
    n = 4;  % x,y,z,psi
    dt_sample = 0.1;  % Sampling interval (seconds)
    
    x_trajec = [];
    y_trajec = [];
    z_trajec = [];
    
    for i=1:m
        % Extract coefficients for segment i
        % Solution structure: [segment1 x,y,z,psi coefficients, segment2 x,y,z,psi coefficients, ...]
        % Each segment has n*(order+1) coefficients
        
        % x coefficients
        x_coef = solution((i-1)*n*(order+1)+1+0*(order+1):(i-1)*n*(order+1)+(order+1)+0*(order+1));
        % y coefficients
        y_coef = solution((i-1)*n*(order+1)+1+1*(order+1):(i-1)*n*(order+1)+(order+1)+1*(order+1));
        % z coefficients
        z_coef = solution((i-1)*n*(order+1)+1+2*(order+1):(i-1)*n*(order+1)+(order+1)+2*(order+1));
        
        % Sample in time period [t(i), t(i+1)]
        t_seg = t(i):dt_sample:t(i+1);
        
        % Calculate position
        x_seg = polyval(x_coef, t_seg);
        y_seg = polyval(y_coef, t_seg);
        z_seg = polyval(z_coef, t_seg);
        
        x_trajec = [x_trajec x_seg];
        y_trajec = [y_trajec y_seg];
        z_trajec = [z_trajec z_seg];
    end
    
    % Calculate velocity (numerical differentiation)
    if length(x_trajec) > 1
        vx_trajec = gradient(x_trajec) / dt_sample;
        vy_trajec = gradient(y_trajec) / dt_sample;
        vz_trajec = gradient(z_trajec) / dt_sample;
    else
        vx_trajec = zeros(size(x_trajec));
        vy_trajec = zeros(size(y_trajec));
        vz_trajec = zeros(size(z_trajec));
    end
    
    % Assemble trajectory [x,y,z, vx,vy,vz]
    trajectory = [x_trajec(:), y_trajec(:), z_trajec(:), ...
                  vx_trajec(:), vy_trajec(:), vz_trajec(:)];
    
    fprintf('  Extracted trajectory: %d points, %.1f seconds\n', size(trajectory, 1), t(end));
end