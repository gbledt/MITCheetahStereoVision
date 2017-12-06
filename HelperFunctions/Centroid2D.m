function [centroid] = Centroid2D(polygon)
    % N x 2 matrix
    [numRow, numCol] = size(polygon);
    xsum = sum(polygon(:,1));
    ysum = sum(polygon(:,2));
    centroid = [xsum / numRow, ysum / numRow];
end