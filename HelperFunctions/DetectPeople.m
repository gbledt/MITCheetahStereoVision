function dispFrame = DetectPeople(peopleDetector, frame_data, point_data)

% Detect people
bboxes = peopleDetector.step(frame_data.frameLeftGray);

if ~isempty(bboxes)
    % Find the centroids of detected people.
    centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
        round(bboxes(:, 2) + bboxes(:, 4) / 2)];
    
    % Find the 3-D world coordinates of the centroids.
    centroidsIdx = sub2ind(size(point_data.disparityMap), ...
        centroids(:, 2), centroids(:, 1));
    X = point_data.points3D(:, :, 1);
    Y = point_data.points3D(:, :, 2);
    Z = point_data.points3D(:, :, 3);
    centroids3D = [X(centroidsIdx), Y(centroidsIdx), Z(centroidsIdx)];
    
    % Find the distances from the camera in meters.
    dists = sqrt(sum(centroids3D .^ 2, 2));
    
    % Display the detect people and their distances.
    labels = cell(1, numel(dists));
    for i = 1:numel(dists)
        labels{i} = sprintf('%0.2f meters', dists(i));
    end
    dispFrame = insertObjectAnnotation(frame_data.frameLeftRect, ...
        'rectangle', bboxes, labels);
else
    dispFrame = frame_data.frameLeftRect;
end