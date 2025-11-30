function collide = collision_check(p, margin)
% collision_check Check for collisions between points in 3D space
%
% Description:
%   Performs collision detection between multiple points in 3D space by 
%   checking if any two points are closer than a specified safety margin.
%   Uses an ellipsoidal distance metric where the z-axis is scaled by 1/3,
%   making collisions more sensitive to horizontal proximity than vertical.
%
% Input Parameters:
%   p      - Point coordinates [N×3] matrix, where each row is [x, y, z]
%   margin - Safety margin distance threshold (meters)
%
% Output Parameters:
%   collide - Binary collision indicator:
%             1: collision detected (at least one pair closer than 2*margin)
%             0: no collision detected
%
% Algorithm:
%   1. Scales z-coordinates by 1/3 to create ellipsoidal distance metric
%   2. Computes pairwise Euclidean distances between all points
%   3. Checks if minimum distance is less than 2 * margin
%

collide = 0;
if(size(p,1) <= 1)
    return;
end
p(:,3) = p(:,3)/3; % scale z-axis by 3 to make it ellipsoid
dis = pdist(p);
if min(dis) < 2*margin
    collide = 1;
end