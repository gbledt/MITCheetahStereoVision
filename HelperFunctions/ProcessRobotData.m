function point_data = ProcessRobotData(point_data)

gridStep = 0.05;
point_data.point_cloud = pcdownsample(point_data.point_cloud,'gridAverage',gridStep);

% Position of the midpoint between the cameras in robot frame
p_camera_robot_frame = [0.6;0;0.2]./2;

% Transform from the camera 3D frame to robot frame
H = [[0,0,1;...
    -1,0,0;...
    0,-1,0;...
    zeros(1,3)],...
    [p_camera_robot_frame;1]];

% Create a rotation matrix
roll = 0*pi/180;
pitch = 0*pi/180;
yaw = -22.5*pi/180;
dx = 0;
dy = 0.6;
dz = 0;
H_shift = [[eul2rotm([yaw,pitch,roll],'ZYX'),[dx;dy;dz]];[0,0,0,1]];


point_data.points_robot_frame = (H_shift*H*[point_data.point_cloud.Location';...
    ones(1,size(point_data.point_cloud.Location,1))])';

% Set bounds for how wide in front to search
lb = [0.5;-1;-1.5];
ub = [10;0.8;2.5];

% Find the cropping limit indices
i = (point_data.points_robot_frame(:,1) >= lb(1) &...
    point_data.points_robot_frame(:,1) < ub(1) & ...
    point_data.points_robot_frame(:,2) >= lb(2) &...
    point_data.points_robot_frame(:,2) < ub(2) & ...
    point_data.points_robot_frame(:,3) >= lb(3) &...
    point_data.points_robot_frame(:,3) < ub(3));

% Crop the points outside of the necessary field of view
point_data.points_robot_frame_crop = point_data.points_robot_frame(i,:);

% figure(11)
% cla
% % plot3(point_data.points_robot_frame(:,1),point_data.points_robot_frame(:,2),point_data.points_robot_frame(:,3),'.','MarkerSize',0.5)
% hold on;
% plot3(point_data.points_robot_frame_crop(:,1),point_data.points_robot_frame_crop(:,2),point_data.points_robot_frame_crop(:,3),'r.','MarkerSize',10)
% 
% axis([0,8,-3,3,-3,3])
% axis square