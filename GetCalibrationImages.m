% Gets a set of image pairs and saves them to folders for the 
% calibration app to use.
GRID_DIMENSION = 22.15;
GRID_UNITS = 'millimeters';

global vid_left;
global vid_right;
global image_index;
vid_left = webcam(2);
vid_right = webcam(1);

NUM_IMAGES = 20;
image_index = 7;

figure('Name', 'Image capture'); 
btn = uicontrol('Style', 'pushbutton', 'String', 'Capture',...
        'Position', [20 20 50 20],...
        'Callback', @capture);
preview(vid_left);

function capture(source, event)
    disp('Taking snapshot!');
    global vid_left;
    global vid_right;
    global image_index;
    img_left = snapshot(vid_left);
    img_right = snapshot(vid_right);
    disp(i);
    figure, imshowpair(img_left, img_right, 'montage'), title('Image pair');

    % Save images to disk.
    left_file = sprintf('Data/left/image_%d.jpg', image_index);
    right_file = sprintf('Data/right/image_%d.jpg', image_index);

    imwrite(img_left, left_file);
    imwrite(img_right, right_file);
    image_index = image_index + 1;
end

% % Run the command below to do the calibation.
% stereoCameraCalibrator('images/left','images/right',GRID_DIMENSION, GRID_UNITS);