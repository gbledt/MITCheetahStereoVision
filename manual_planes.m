%% ========= Stereo Params =========%%
BLOCK_SIZE = 15;
DISPARITY_RANGE = [0 48];
CONTRAST_THRESHOLD = 0.5;
UNIQUENESS_THRESHOLD = 15;
TEXTURE_THRESHOLD = 0.0004;
MATCH_CONTRAST = false;
%% ================================%%
img1 = 'Data/left/image_1.jpg';
img2 = 'Data/right/image_1.jpg';
img1 = imread(img1);
img2 = imread(img2);

frameLeft = img1;
frameRight = img2;

% Rectify the frames.
[frameLeftRect, frameRightRect] = rectifyStereoImages(frameLeft, frameRight, stereoParams);
figure, imshowpair(frameLeftRect, frameRightRect, 'montage');

% Convert to grayscale.
frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

% Get edges from left image
EDGE_THRESHOLD = [0.05 0.2];
EDGE_SIGMA = 11;
edge_map = edge(frameLeftGray, 'Canny', EDGE_THRESHOLD, EDGE_SIGMA);
figure, imshowpair(frameLeftRect, edge_map, 'montage');

% Compute disparity.
disparityMap = disparity(frameLeftGray, frameRightGray,...
    'Method', 'SemiGlobal',...
    'BlockSize', BLOCK_SIZE, ...
    'DisparityRange', DISPARITY_RANGE, ...
    'ContrastThreshold', CONTRAST_THRESHOLD, ...
    'UniquenessThreshold', UNIQUENESS_THRESHOLD);

% Mask out the background
mask = zeros(size(frameLeftGray));
mask(25:end-25,25:end-25) = 1;
final_mask = activecontour(frameLeftGray, mask, 100);
disparityMap = disparityMap .* final_mask;

figure, imshow(disparityMap, DISPARITY_RANGE);
title('Disparity Map');
colormap(gca,jet) 
colorbar

% Reconstruct 3-D scene.
points3D = reconstructScene(disparityMap, stereoParams);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');
view(player3D, ptCloud);

fullCloud = pointCloud([0,0,0]);
for ii = 1:2
    [bw, xi, yi] = roipoly(frameLeftGray);
    smoothSurface = SmoothMaskedSurface(bw, ptCloud);
    fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);
end

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');
view(player3D, fullCloud);
% hold on;
% plot(plane_model);

function [plane_model, ptCloudSeg] = fitPlane(mask, ptCloud)
    indices = find(mask);
    ptCloudSeg = select(ptCloud, indices);
    [plane_model,inlierIndices,outlierIndices] = pcfitplane(ptCloudSeg, 0.03);
end