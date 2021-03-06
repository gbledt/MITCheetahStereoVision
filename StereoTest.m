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
options.DATA_FILE = true&~options.LIVE_STREAM;
options.DATA_OPT = 1;
options.RECEIVE_LCM = false;
options.BROADCAST_LCM = false;
options.LOOP_RATE = 1;  % Hz

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
    right_cam = webcam(1);
    
    % Load the calibration data
    data_string = 'stereoParams';
    load([data_string,'.mat']);
end

% Load data from a file
if options.DATA_FILE
    
    % Case structure to select the data to be used
    switch options.DATA_OPT
        case 0
            data_string = 'handshakeStereo';
            
            % Get the data from the left and right video
            videoFileLeft = [data_string,'_left.avi'];
            videoFileRight = [data_string,'_right.avi'];
            
            % Set up video readers
            readerLeft = vision.VideoFileReader(videoFileLeft, ...
                'VideoOutputDataType', 'uint8');
            readerRight = vision.VideoFileReader(videoFileRight, ...
                'VideoOutputDataType', 'uint8');
            
        case 1
            data_string = 'stereo';
            
    end
    
    % Load the stereoParameters object.
    load([data_string,'Params.mat']);
    
end

% Set up the people detector object
if options.PEOPLE_DETECTOR
    peopleDetector = vision.PeopleDetector('MinSize', [166 83]);
end

% Initialize the LCM publisher
if options.BROADCAST_LCM
    % Create the LCM instance
    lc = lcm.lcm.LCM.getSingleton();
    aggregator = lcm.lcm.MessageAggregator();
    Msg = cheetahlcm.vision_data_t();
end

% Timing
loop_timer = tic;
t0 = toc(loop_timer);

%% ========================== Run the Loop ================================

% Run the loop until the data is done or live stream is ended
while options.RUN_VISION && (options.LIVE_STREAM ||...
        (options.DATA_FILE && ...
        (~options.DATA_OPT == 0 || (~isDone(readerLeft) && ~isDone(readerRight)))))
    
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
        switch options.DATA_OPT
            case 0
                frame_data.frameLeft = readerLeft.step();
                frame_data.frameRight = readerRight.step();
            case 1
                frame_data.frameLeft = imread('Data/left/image_1.jpg');
                frame_data.frameRight = imread('Data/right/image_1.jpg');
        end
    end
    
    % Process the relevant frame data
    frame_data = ProcessFrames(frame_data, stereoParams);
    
    % Get the 3D point data
    point_data = CalcStereoPointCloud(frame_data, stereoParams);
    
    % View the point cloud
    view(point_cloud_viewer, point_data.point_cloud);
    
    % Gets object data from the point cloud
    point_data = ProcessRobotData(point_data);
    
    % Gets object data from the point cloud
    %     object_data = GetObjectData(point_data);
    
    % Show the distance to the nearest person if there is one
    if options.PEOPLE_DETECTOR
        % Detect people and anotate the frame
        dispFrame = DetectPeople(peopleDetector, frame_data, point_data);
        
        % Display the frame.
        step(player, dispFrame);
    end
    
    % Broadcast out the LCM messages containing object data
    if options.BROADCAST_LCM
        lc.publish('CHEETAH_vision_data', visionMsg);
    end
    
    while ((toc(loop_timer) - t0) < 1/options.LOOP_RATE),end;
    
    t0 = toc(loop_timer);
    if options.DATA_FILE && options.DATA_OPT == 1
        options.RUN_VISION = false;
    end
end

%% ========================= Clean Up and Exit ============================
% Clean up
if options.LIVE_STREAM
    clear('right_cam');
    clear('left_cam');
end

if options.DATA_FILE
    if (options.DATA_OPT == 0)
        reset(readerLeft);
        reset(readerRight);
    end
end

release(player);