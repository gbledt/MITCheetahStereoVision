function frame_data = ProcessFrames(frame_data, stereoParams)

% Rectify the frames
[frame_data.frameLeftRect, frame_data.frameRightRect] = ...
    rectifyStereoImages(frame_data.frameLeft, frame_data.frameRight, stereoParams);

% Convert to grayscale
frame_data.frameLeftGray  = rgb2gray(frame_data.frameLeftRect);
frame_data.frameRightGray = rgb2gray(frame_data.frameRightRect);