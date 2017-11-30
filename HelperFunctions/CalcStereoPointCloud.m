function point_data = CalcStereoPointCloud(frame_data, stereoParams)

% Compute disparity
point_data.disparityMap = disparity(frame_data.frameLeftGray, frame_data.frameRightGray);

% Reconstruct 3-D scene
points3D = reconstructScene(point_data.disparityMap, stereoParams);
point_data.points3D = points3D ./ 1000;
point_cloud = pointCloud(point_data.points3D, ...
    'Color', frame_data.frameLeftRect);

[point_cloud, indices] = removeInvalidPoints(point_cloud);
% gridStep = 0.01;
% point_data.point_cloud = pcdownsample(point_data.point_cloud,'gridAverage',gridStep);
% point_data.point_cloud = pcdenoise(point_data.point_cloud);

point_data.point_cloud = point_cloud;