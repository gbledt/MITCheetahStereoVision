% Gets a set of image pairs and saves them to folders for the 
% calibration app to use.

% Run stereoCameraCalibrator to get to the app.

vid_left = videoinput('winvideo', 2);
vid_right = videoinput('winvideo', 3);

NUM_IMAGES = 20;
preview(vid_left);

for i = 1:NUM_IMAGES

        img_left = getsnapshot(vid_left);
        img_right = getsnapshot(vid_right);
        figure, imshowpair(img_left, img_right, 'montage'), title('Image pair');
        
        % Save images to disk.
        left_file = sprintf('images/left/image_%d.jpg', i);
        right_file = sprintf('images/right/image_%d.jpg', i);

        imwrite(img_left, left_file);
        imwrite(img_right, right_file);
        
        delete(img_left);
        delete(img_right);

        pause(5);
        close all;
end