function [dist] = AvgPlaneDistance(points, plane)
% points: Nx3 matrix
% plane: 4 element vector
    distances = abs(points * transpose(plane(1,1:3)) + plane(4));
    dist = mean(distances);
end

