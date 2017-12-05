% Extracts the best contours from image and smooths.

%% ==== Parameters ==== %%
THETA_X_DEG = 12;
THETA_X_RAD = degtorad(THETA_X_DEG);
tform_x = affine3d(makehgtform('xrotate', THETA_X_RAD));

THETA_Z_DEG = 5;
THETA_Z_RAD = degtorad(THETA_Z_DEG);
tform_z = affine3d(makehgtform('zrotate', THETA_Z_RAD));

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

middleSlicePlane = planeModel([1, 0, 0, 0]);
leftSlicePlane = planeModel([1, 0, 0, 0.6]);
rightSlicePlane = planeModel([1, 0, 0, -0.6]);

vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.
%% ==================== %%

bestPoly = LargestContours(frameLeftGray, 2, 9);
plot(bestPoly(1).X, bestPoly(1).Y);
hold on;
for ii = 2:9
    plot(bestPoly(ii).X, bestPoly(ii).Y);
end

[HEIGHT, WIDTH] = size(disparityMap);
fullCloud = pointCloud([0,0,0]);

% Detect and store all horizontal planes.
maxHorizPlanes = 10;
numHorizPlanes = 0;
horizPlanesIndex = 1;
horizPlanes = zeros(maxHorizPlanes, 4);

% Detect and store all horizontal planes.
maxVertPlanes = 10;
numVertPlanes = 0;
vertPlanesIndex = 1;
vertPlanes = zeros(maxHorizPlanes, 4);

M = zeros(2, numel(bestPoly));
for ii = 1:numel(bestPoly)
    
    poly = bestPoly(ii);
    poly.X = [poly.X, poly.X(1)];
    poly.Y = [poly.Y, poly.Y(1)];
    bw = poly2mask(poly.X, poly.Y, HEIGHT, WIDTH);
    [smoothSurface, plane_model] = SmoothMaskedSurface(bw, ptCloud);
    fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);
    
    % Score planes based on their dot product against references unit
    % vectors.
    horizScore = abs(dot(plane_model.Normal, horizNorm));
    vertScore = abs(dot(plane_model.Normal, vertNorm))
    thresh = 1.5;
    
    % Check if this is a horizontal plane.
    if (horizScore / vertScore) > thresh
        horizPlanes(horizPlanesIndex,:) = plane_model.Parameters;
        horizPlanesIndex = horizPlanesIndex + 1;
    end
    
    % Also check if a vertical plane.
    if (vertScore / horizScore) > thresh
        vertPlanes(vertPlanesIndex,:) = plane_model.Parameters;
        vertPlanesIndex = vertPlanesIndex + 1;
    end
    
    % Find intersection of plane model with the side plane.
    N1 = plane_model.Normal;
    A1 = [0 0 -plane_model.Parameters(4) / plane_model.Parameters(3)];
    N2 = middleSlicePlane.Normal;
    A2 = [0 0 0];
    [linePoint, lineVector, check]= PlaneIntersect(N1, A1, N2, A2);
    M(:,ii) = [lineVector(3), lineVector(2)];
%     plot(lineVector(2:3));
end

% plotv(M, '-');
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');

% unrotatedCloud = pctransform(fullCloud, tform_x);
% unrotatedCloud = pctransform(unrotatedCloud, tform_z);
view(player3D, fullCloud);

% Project full cloud on to side plane.
% [projectedPts, newCloud] = ProjectToPlane(unrotatedCloud, middleSlicePlane);
% view(player3D, newCloud);