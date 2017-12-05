function [smoothSurface, plane_model] = SmoothPointCloudRegion(mask, ptCloud)
    % Get the indices of points inside mask.
    indices = find(mask);
    ptCloudSeg = select(ptCloud, indices);
    
    % Fit a plane to the points inside mask.
    [plane_model,inlierIndices,outlierIndices] = pcfitplane(ptCloudSeg, 0.01);
    
    % Project all points on to the best plane.
    points = ptCloudSeg.Location;
    distances = points * transpose(plane_model.Parameters(1,1:3)) + plane_model.Parameters(1,4);
    projected = points - distances * plane_model.Normal;
    smoothSurface = pointCloud(projected);
end


