%% ======================== MIT Cheetah Vision ============================
% Gerardo Bledt
% Milo Knowles
% 6.869: Advances in Computer Vision
% Final Project
%
% Dynamic Locomotion Terrain Recognition​for​the MIT​Cheetah​3 ​Robot
%
clear; close all; clc

%% ============================== Options =================================

% Options
options.RUN_VISION = true;
options.LIVE_STREAM = true;
options.PEOPLE_DETECTOR = false;
options.DATA_FILE = true&~options.LIVE_STREAM;
options.DATA_OPT = 0;
options.RECEIVE_LCM = false;
options.BROADCAST_LCM = false;

%% =============================== Setup ==================================

% Video player object to display video
player = vision.DeployableVideoPlayer('Location', [20, 400]);

% Create a streaming point cloud viewer
point_cloud_viewer = pcplayer([-30, 30], [-30, 30], [0, 80], ...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

% Setup the live stream for the webcams
if options.LIVE_STREAM
    % Create the right and left webcam objects
    left_cam = webcam(1);
    right_cam = webcam(2);
    
    % Load the calibration data
    data_string = 'stereoParams';
    load([data_string,'.mat']);
end

% Load data from a file
if options.DATA_FILE
    
    % Case structure to select the data to be used
    switch options.DATA_OPT
        case 0
            data_string = 'handshake';
    end
    
    % Load the stereoParameters object.
    load([data_string,'StereoParams.mat']);
    
    % Get the data from the left and right video
    videoFileLeft = [data_string,'_left.avi'];
    videoFileRight = [data_string,'_right.avi'];
    
    % Set up video readers
    readerLeft = vision.VideoFileReader(videoFileLeft, ...
        'VideoOutputDataType', 'uint8');
    readerRight = vision.VideoFileReader(videoFileRight, ...
        'VideoOutputDataType', 'uint8');
end

% Set up the people detector object
if options.PEOPLE_DETECTOR
    peopleDetector = vision.PeopleDetector('MinSize', [166 83]);
end

%% ========================== Run the Loop ================================

% Run the loop until the data is done or live stream is ended
while (options.RUN_VISION && options.LIVE_STREAM) ||...
        (options.DATA_FILE && (~isDone(readerLeft) && ~isDone(readerRight)))
    
    % Receive LCM options from main GUI
    if options.RECEIVE_LCM
        
    end
    
    % Read the frames from the webcams
    if options.LIVE_STREAM
        frame_data.frameLeft = snapshot(left_cam);
        frame_data.frameRight = snapshot(right_cam);
        %imshow(frame_data.frameLeft)
        %imshow(frame_data.frameRight)
    end
    
    % Read the frames from the data file
    if options.DATA_FILE
        frame_data.frameLeft = readerLeft.step();
        frame_data.frameRight = readerRight.step();
    end
    
    % Process the relevant frame data
    frame_data = ProcessFrames(frame_data, stereoParams);
    
    % Get the 3D point data
    point_data = CalcStereoPointCloud(frame_data, stereoParams);
    
    % View the point cloud
    view(point_cloud_viewer, point_data.point_cloud);
    
    % Gets object data from the point cloud
    object_data = GetObjectData(point_data);
    
    % Show the distance to the nearest person if there is one
    if options.PEOPLE_DETECTOR
        % Detect people and anotate the frame
        dispFrame = DetectPeople(peopleDetector, frame_data, point_data);
        
        % Display the frame.
        step(player, dispFrame);
    end
    
    % Broadcast out the LCM messages containing object data
    if options.BROADCAST_LCM
        
    end
end

%% ========================= Clean Up and Exit ============================
% Clean up
if options.LIVE_STREAM
    clear('right_cam');
    clear('left_cam');
end

if options.DATA_FILE
    reset(readerLeft);
    reset(readerRight);
end

release(player);