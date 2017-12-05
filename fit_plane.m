% Extracts the best contours from image and smooths.

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
groundPlane = planeModel([0 -1 0, 0]);

vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.

% unrotatedCloud = pctransform(fullCloud, tform_x);
% unrotatedCloud = pctransform(unrotatedCloud, tform_z);
%% ========================================= %%

%% ================= Contours =============== %% 
bestPoly = LargestContours(frameLeftGray, 2, 7);
% plot(bestPoly(1).X, bestPoly(1).Y);
% hold on;
% for ii = 2:9
%     plot(bestPoly(ii).X, bestPoly(ii).Y);
% end

[HEIGHT, WIDTH] = size(disparityMap);
fullCloud = pointCloud([0,0,0]);

% Detect and store all horizontal planes.
horizPlanes = [];

% Detect and store all horizontal planes.
vertPlanes = [];

M = zeros(2, numel(bestPoly));
for ii = 1:numel(bestPoly)
    % Fit a plane to each segmented region of the scene.
    poly = bestPoly(ii);
    poly.X = [poly.X, poly.X(1)];
    poly.Y = [poly.Y, poly.Y(1)];
    bw = poly2mask(poly.X, poly.Y, HEIGHT, WIDTH);
    [smoothSurface, plane_model] = SmoothMaskedSurface(bw, ptCloud);
    fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);
    
    % Score planes based on their dot product against references unit
    % vectors.
    horizScore = abs(dot(plane_model.Normal, horizNorm));
    vertScore = abs(dot(plane_model.Normal, vertNorm));
    thresh = 1.5;
    
    % Check if this is a horizontal plane.
    if (horizScore / vertScore) > thresh
        horizPlanes = [horizPlanes; plane_model.Parameters];
    end
    
    % Also check if a vertical plane.
    if (vertScore / horizScore) > thresh
        vertPlanes = [vertPlanes; plane_model.Parameters];
    end
end

%%  Find intersection of vertical planes with the ground. %%
heightTransPts = [];
[vpRow, vpCol] = size(vertPlanes);
for ii = 1:vpRow
    vp = vertPlanes(ii, :);
    N1 = vp(1:3);
    A1 = [0 0 -vp(4) / vp(3)];
    
    N2 = groundPlane.Normal;
    A2 = [0 0 0];
    
    [linePoint, lineVector, check]= PlaneIntersect(N1, A1, N2, A2);
    intPoint = LinePlaneIntersect(lineVector, linePoint, groundPlane.Parameters);
    heightTransPts = [heightTransPts, intPoint(3)];
end

%% Find the active horizontal plane between transition pts %%
heightTransPts = [0, heightTransPts, 5]; % Add foreground and background pt
activeHorizPlanes = zeros(numel(heightTransPts)-1, 4);
[hpRow, hpCol] = size(horizPlanes);

% For each z-region (between transition points)
% Find the plane that best fits points in that region.
for zz = 1:(numel(heightTransPts)-1)
    roi = [-0.3 0.3; -2 2; heightTransPts(zz) heightTransPts(zz+1)];
    pointsInROI = findPointsInROI(fullCloud, roi);
    sample = datasample(pointsInROI, 1000);
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

activeHeights = zeros(numel(heightTransPts)-1);
Z = [];
Y = [];
for ahp = 1:(numel(heightTransPts)-1)
    plane = activeHorizPlanes(ahp,:);
    min_z = heightTransPts(ahp);
    max_z = heightTransPts(ahp+1);
    
    N1 = midSlicePlane.Normal;
    A1 = [0 0 0];
    N2 = plane(1:3);
    A2 = [0 0 -plane(4) / plane(3)];
    [linePt, lineVect, check] = PlaneIntersect(N1, A1, N2, A2);
    Tmin = (min_z - linePt(3)) / lineVect(3);
    Tmax = (max_z - linePt(3)) / lineVect(3);
    minPt = linePt + Tmin * lineVect
    maxPt = linePt + Tmax * lineVect
    Z = [Z, minPt(3), maxPt(3)];
    Y = [Y, minPt(2), maxPt(2)];
end

figure, plot(Z, -1 * Y);

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');
view(player3D, fullCloud);

% Project full cloud on to side plane.
% [projectedPts, newCloud] = ProjectToPlane(fullCloud, middleSlicePlane);
% view(player3D, newCloud);