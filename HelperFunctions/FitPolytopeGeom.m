function [sliceMap, poly3d] = FitPolytopeGeom(ptCloud, THETA_X_DEG, THETA_Z_DEG, NUM_HORIZONTAL, NUM_VERTICAL)
    %% ============= Parameters ============= %%
    THETA_X_RAD = degtorad(THETA_X_DEG);
    tform_x = affine3d(makehgtform('xrotate', THETA_X_RAD));
    THETA_Z_RAD = degtorad(THETA_Z_DEG);
    tform_z = affine3d(makehgtform('zrotate', THETA_Z_RAD));
    
    vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
    horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.

    ptCloud = pctransform(ptCloud, tform_x);
    ptCloud = pctransform(ptCloud, tform_z);
    
    midPlane = planeModel([1, 0, 0, 0]);
    leftPlane = planeModel([1, 0, 0, 0.6]);
    rightPlane = planeModel([1, 0, 0, -0.6]);
    
    groundPlane = planeModel([0 1 0 -0.4]);
    frontPlane = planeModel([0 0 1 -1.0]);
    topPlane = planeModel([0 1 0 0.8]);
    backPlane = planeModel([0 0 1 -2.5]);

    OBS_PADDING = 0.4; % meters in front and behind the obstacle that we care about.
    ROI_SAMPLES = 500; % How many points to use for active plane estimation?
    ROI_X = 1;
    ROI_Y = 2;
    %% Fit horizontal and vertical planes to the cloud. %%
    [allPlanes, horizPlanes, vertPlanes, inlierCloud] = FitPlanesGeom(ptCloud, NUM_HORIZONTAL, NUM_VERTICAL);
    
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
    vertPlanes = vertPlanes(ix,:);
%     heightTransPts = [heightTransPts(1)-OBS_PADDING, heightTransPts, heightTransPts(end)+OBS_PADDING];
    
    activeHorizPlanes = zeros(numel(heightTransPts)-1, 4);
    [hpRow, hpCol] = size(horizPlanes);

    % For each z-region (between transition points)
    % Find the plane that best fits points in that region.
    % This is the 'active' plane in that region.
    horizPlaneScores = [];
    for zz = 1:(numel(heightTransPts)-1)
        roi = [-ROI_X ROI_X; -ROI_Y ROI_Y; heightTransPts(zz) heightTransPts(zz+1)];
        pointsInROI = findPointsInROI(inlierCloud, roi);
        sample = datasample(pointsInROI, ROI_SAMPLES);
        samplePts = select(inlierCloud, sample);

        bestScore = inf;
        bestPlane = 1;
        for pp = 1:hpRow
            score = DistPointPlane(samplePts.Location, horizPlanes(pp,:)) / ROI_SAMPLES;
            if score < bestScore
                bestPlane = pp;
                bestScore = score;
            end
        end
        activeHorizPlanes(zz,:) = horizPlanes(bestPlane,:);
        horizPlaneScores = [horizPlaneScores, bestScore];
    end

    horizPlaneScores
    heightTransPts
    % Combine the vertical and horiz planes.
    % For N vert planes, we have N-1 horizontal planes.
    planeList = [];
    for ii = 1:(vpRow-1)
        planeList = [planeList; vertPlanes(ii,:); activeHorizPlanes(ii,:)];
    end
    planeList = [planeList; vertPlanes(ii+1,:)];
    
    % Logic to determine bounding planes.
    lastPlane = planeList(end,:);
    horizScore = abs(dot(lastPlane(1:3), horizNorm));
    vertScore = abs(dot(lastPlane(1:3), vertNorm));
    
    % If last plane horizontal, cut off with vertical plane.
    if (horizScore / vertScore) > 1.5
        planeList = [planeList; backPlane.Parameters]
    else
        planeList = [planeList; topPlane.Parameters]
    end
    
    firstPlane = planeList(1,:);
    horizScore = abs(dot(firstPlane(1:3), horizNorm));
    vertScore = abs(dot(firstPlane(1:3), vertNorm));
    
    if (horizScore / vertScore) > 1.5
        planeList = [frontPlane.Parameters ; planeList];
    else
        planeList = [groundPlane.Parameters ; planeList];
    end
    
    % Get boundaries for each plane.
    % Front plane is horizontal, back plane is vertical
    [apRow, apCol] = size(planeList);

    % For each plane, make a polygon using the plane before and after it.
    poly3d = zeros(apRow-2, 4, 3); % each polygon has 5 XYZ points

    % Slice map stores zy pairs where z is depth and y is height.
    sliceMap = [];
    
    for ii = 2:apRow-1
        befPlane = planeList(ii-1,:);
        plane = planeList(ii,:);
        aftPlane = planeList(ii+1,:);

        % Get point and normal of plane. 
        N1 = plane(1:3);
        A1 = GetPointOnPlane(plane);

        % Line caused by intersection with plane before
        N_bef = befPlane(1:3);
        A_bef = GetPointOnPlane(befPlane);
        [bPt, bVect, check] = PlaneIntersect(N1, A1, N_bef, A_bef);
        lPt_bef = LinePlaneIntersect(bVect, bPt, leftPlane.Parameters);
        mPt_bef = LinePlaneIntersect(bVect, bPt, midPlane.Parameters);
        rPt_bef = LinePlaneIntersect(bVect, bPt, rightPlane.Parameters);

        % Line caused by intersection with plane after
        N_aft = aftPlane(1:3);
        A_aft = GetPointOnPlane(aftPlane);
        [aPt, aVect, check] = PlaneIntersect(N1, A1, N_aft, A_aft);
        lPt_aft = LinePlaneIntersect(aVect, aPt, leftPlane.Parameters);
        mPt_aft = LinePlaneIntersect(aVect, aPt, midPlane.Parameters);
        rPt_aft = LinePlaneIntersect(aVect, aPt, rightPlane.Parameters);

        % Now we have 4 points to define a polygon with.
        poly3d(ii-1,:,:) = [lPt_bef; rPt_bef; rPt_aft; lPt_aft];
        
        % Add to slice map;
        sliceMap = [sliceMap; mPt_bef(3), -mPt_bef(2); mPt_aft(3), -mPt_aft(2)];
    end
    planeList
end

