function object_data = GetObjectData(point_data)
% We will need to add in the logic to deal with the
% point cloud data and find stair heights here

% Position of the midpoint between the cameras in robot frame
p_camera_robot_frame = [0.6;0;0.2]./2;

% 
point_data.points_robot_frame = [p_camera_robot_frame(1) + point_data.point_cloud.Location(:,3),...
    p_camera_robot_frame(2) - point_data.point_cloud.Location(:,1),...
    p_camera_robot_frame(3) - point_data.point_cloud.Location(:,2)];


%plot3(point_data.points_robot_frame(:,1),point_data.points_robot_frame(:,2),point_data.points_robot_frame(:,3),'.')
%axis([0,8,-3,3,-3,3])

object_data = [];