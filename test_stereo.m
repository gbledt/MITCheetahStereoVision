%% ========= Stereo Params =========%%
BLOCK_SIZE = 15;
DISPARITY_RANGE = [0 48];
CONTRAST_THRESHOLD = 0.5;
UNIQUENESS_THRESHOLD = 15;
TEXTURE_THRESHOLD = 0.0004;
MATCH_CONTRAST = false;

%% ================================%%
load('stereoParams.mat');
img1 = 'Data/left/image_8.jpg';
img2 = 'Data/right/image_8.jpg';
img1 = imread(img1);
img2 = imread(img2);

% figure, imshowpair(img1, img2, 'montage');
frameLeft = img1;
frameRight = img2;

% Rectify the frames.
[frameLeftRect, frameRightRect] = rectifyStereoImages(frameLeft, frameRight, stereoParams);

figure, imshowpair(frameLeftRect, frameRightRect, 'montage');
% figure, imshow(stereoAnaglyph(frameLeftRect, frameRightRect));

% Convert to grayscale.
frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

% Contrast normalize the second image to match the first.
frameRightGray = histeq(frameRightGray, imhist(frameLeftGray));

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