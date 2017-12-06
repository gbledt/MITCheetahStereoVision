% Extracts the best contours from image and smooths.
close all;
%% ============= Parameters ============= %%
THETA_X_DEG = 12;
THETA_X_RAD = degtorad(THETA_X_DEG);
tform_x = affine3d(makehgtform('xrotate', THETA_X_RAD));

THETA_Z_DEG = 5;
THETA_Z_RAD = degtorad(THETA_Z_DEG);
tform_z = affine3d(makehgtform('zrotate', THETA_Z_RAD));

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

midSlicePlane = planeModel([1, 0, 0, 0]);
leftSlicePlane = planeModel([1, 0, 0, 0.6]);
rightSlicePlane = planeModel([1, 0, 0, -0.6]);
groundPlane = planeModel([0 -1 0 0.4]);

vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.

% ptCloud = pctransform(ptCloud, tform_x);
% ptCloud = pctransform(ptCloud, tform_z);
NUM_CONTOURS = 6;
NUM_CONTOUR_LVL = 2;

OBS_PADDING = 0.4; % meters in front and behind the obstacle that we care about.
ROI_SAMPLES = 500; % How many points to use for active plane estimation?
ROI_X = 1;
ROI_Y = 2;
%% ========================================= %%

%% ================= Contours =============== %%
bestPoly = LargestContours(frameLeftGray, NUM_CONTOUR_LVL, NUM_CONTOURS);

% Plot the largest, non-overlapping contours from image.
figure, plot(bestPoly(1).X, bestPoly(1).Y);
hold on;
for ii = 2:NUM_CONTOURS
    plot(bestPoly(ii).X, bestPoly(ii).Y);
end
axis ij;

[HEIGHT, WIDTH] = size(disparityMap);
[fullCloud, horizPlanes, vertPlanes, allPlanes] = PlanarizePointCloud(ptCloud, bestPoly, WIDTH, HEIGHT);

%%  Find intersection of vertical planes with the ground. %%
heightTransPts = []; % Stores points where the height of the ground is likely to change.
[vpRow, vpCol] = size(vertPlanes);
for ii = 1:vpRow
    vp = vertPlanes(ii, :); % Extract parameters from this vertical plane.
    N1 = vp(1:3);
    A1 = GetPointOnPlane(vp);
    
    N2 = groundPlane.Normal; % Get the parameters from ground plane.
    A2 = GetPointOnPlane(groundPlane.Parameters);

    % Find line where the vertical plane hits ground.
    [linePoint, lineVector, check]= PlaneIntersect(N1, A1, N2, A2);
    
    % Get the point where this line intersects ground (coplanar in this
    % case).
    intPoint = LinePlaneIntersect(lineVector, linePoint, groundPlane.Parameters);
    
    % The z-coord is the distance of that vertical plane from camera.
    heightTransPts = [heightTransPts, intPoint(3)];
end

%% Find the active horizontal plane between transition pts %%
 % Add foreground and background pt
[b, ix] = sort(heightTransPts);
heightTransPts = heightTransPts(ix); % Make sure these are in order.

heightTransPts = [heightTransPts(1)-OBS_PADDING, heightTransPts, heightTransPts(numel(heightTransPts))+OBS_PADDING];
activeHorizPlanes = zeros(numel(heightTransPts)-1, 4);
[hpRow, hpCol] = size(horizPlanes);

% For each z-region (between transition points)
% Find the plane that best fits points in that region.
% This is the 'active' plane in that region.
for zz = 1:(numel(heightTransPts)-1)
    roi = [-ROI_X ROI_X; -ROI_Y ROI_Y; heightTransPts(zz) heightTransPts(zz+1)]
    pointsInROI = findPointsInROI(fullCloud, roi);
    sample = datasample(pointsInROI, ROI_SAMPLES);
    samplePts = select(fullCloud, sample);
    
    bestScore = inf;
    bestPlane = 1;
    for pp = 1:hpRow
        score = DistPointPlane(samplePts.Location, horizPlanes(pp,:));
        if score < bestScore
            bestPlane = pp;
            bestScore = score;
        end
    end
    activeHorizPlanes(zz,:) = horizPlanes(bestPlane,:);
end

% Get a side view of the active heights.
activeHeights = zeros(numel(heightTransPts)-1);
Z = [];
Y = [];
for ahp = 1:(numel(heightTransPts)-1)
    plane = activeHorizPlanes(ahp,:);
    min_z = heightTransPts(ahp);
    max_z = heightTransPts(ahp+1);
    
    N1 = midSlicePlane.Normal;
    A1 = GetPointOnPlane(midSlicePlane.Parameters);
    
    N2 = plane(1:3);
    A2 = GetPointOnPlane(plane);

    [linePt, lineVect, check] = PlaneIntersect(N1, A1, N2, A2);
    Tmin = (min_z - linePt(3)) / lineVect(3);
    Tmax = (max_z - linePt(3)) / lineVect(3);
    minPt = linePt + Tmin * lineVect;
    maxPt = linePt + Tmax * lineVect;
    Z = [Z, minPt(3), maxPt(3)];
    Y = [Y, minPt(2), maxPt(2)];
end

% Plot side view.
figure, plot(Z, -1 * Y);

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');
view(player3D, fullCloud);
