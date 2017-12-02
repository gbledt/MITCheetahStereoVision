% Place image pairs in images/left/ and images/right/ then run this.
left_img_dir = 'images/left/';
right_img_dir = 'images/right/';

image_left_info = dir(fullfile(left_img_dir, '*.jpg'));
image_right_info = dir(fullfile(right_img_dir, '*.jpg'));

imageFileNames1 = fullfile(left_img_dir, {image_left_info.name});
imageFileNames2 = fullfile(right_img_dir, {image_right_info.name});

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
SQUARE_DIMENSION = 22.15;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, SQUARE_DIMENSION);

% Read one of the images from the first stereo pair
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(stereoParams);

% Visualize pattern locations
h2=figure; showExtrinsics(stereoParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, stereoParams);

% You can use the calibration data to rectify stereo images.
I2 = imread(imageFileNames2{1});
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

% Save the stereo params.
fname = 'stereo_params.mat';
save(fname, 'stereoParams');
