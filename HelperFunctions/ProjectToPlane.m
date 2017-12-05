function [projected, surface] = ProjectToPlane(ptCloud, planeModel)
    points = ptCloud.Location;
    distances = points * transpose(planeModel.Parameters(1,1:3)) + planeModel.Parameters(1,4);
    projected = points - distances * planeModel.Normal;
    surface = pointCloud(projected);
end

