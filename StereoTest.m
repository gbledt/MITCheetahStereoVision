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
options.LIVE_STREAM = false;
options.PEOPLE_DETECTOR = false;
options.DATA_FILE = true;
options.DATA_OPT = 0;

%% =============================== Setup ==================================

% Video player object to display video
player = vision.DeployableVideoPlayer('Location', [20, 400]);

% Create a streaming point cloud viewer
point_cloud_viewer = pcplayer([-3, 3], [-3, 3], [0, 8], ...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

% Setup the live stream for the webcams
if options.LIVE_STREAM
    % Create the right and left webcam objects
    left_cam = webcam(2);
    right_cam = webcam(3);
end

% Load data from a file
if options.DATA_FILE && ~options.LIVE_STREAM
    
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
    
    % Read the frames from the webcams
    if options.LIVE_STREAM
        frame_data.frameLeft = snapshot(left_cam);
        frame_data.frameRight = snapshot(right_cam);
    end
    
    % Read the frames from the data file
    if options.DATA_FILE && ~options.LIVE_STREAM
        frame_data.frameLeft = readerLeft.step();
        frame_data.frameRight = readerRight.step();
    end
    
    % Process the relevant frame data
    frame_data = ProcessFrames(frame_data, stereoParams);
    
    % Get the 3D point data
    point_data = CalcStereoPointCloud(frame_data, stereoParams);
    
    % View the point cloud
    view(point_cloud_viewer, point_data.point_cloud);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % We will need to add in the logic to deal with the 
    % point cloud data and find stair heights here
    %
    object_data = GetObjectData(point_data);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Show the distance to the nearest person if there is one
    if options.PEOPLE_DETECTOR
        % Detect people and anotate the frame
        dispFrame = DetectPeople(peopleDetector, frame_data, point_data);
        
        % Display the frame.
        step(player, dispFrame);
    end
end

%% ========================= Clean Up and Exit ============================
% Clean up
clear('right_cam');
clear('left_cam');
reset(readerLeft);
reset(readerRight);
release(player);