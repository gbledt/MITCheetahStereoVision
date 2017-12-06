function point_data = CalcStereoPointCloud(frame_data, stereoParams)
%% ====== STEREO PARAMS ==== %%
BLOCK_SIZE = 15;
DISPARITY_RANGE = [0 48];
CONTRAST_THRESHOLD = 0.5;
UNIQUENESS_THRESHOLD = 15;
TEXTURE_THRESHOLD = 0.0004;
MATCH_CONTRAST = false;
%% ========================= %%

% Compute disparity
point_data.disparityMap = disparity(frame_data.frameLeftGray, frame_data.frameRightGray,...
    'Method', 'SemiGlobal',...
    'BlockSize', BLOCK_SIZE, ...
    'DisparityRange', DISPARITY_RANGE, ...
    'ContrastThreshold', CONTRAST_THRESHOLD, ...
    'UniquenessThreshold', UNIQUENESS_THRESHOLD);

% Mask out the background
mask = zeros(size(frame_data.frameLeftGray));
mask(25:end-25,25:end-25) = 1;
final_mask = activecontour(frame_data.frameLeftGray, mask, 100);
point_data.disparityMap = point_data.disparityMap .* final_mask;

% Reconstruct 3-D scene
points3D = reconstructScene(point_data.disparityMap, stereoParams);
point_data.points3D = points3D ./ 1000;
point_cloud = pointCloud(point_data.points3D, ...
    'Color', frame_data.frameLeftRect);

% [point_cloud, ~] = removeInvalidPoints(point_cloud);
% point_data.point_cloud = pcdenoise(point_cloud);

point_data.point_cloud = point_cloud;
end