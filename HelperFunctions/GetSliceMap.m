function [sliceMap, poly3d] = GetSliceMap(frameLeftGray, ptCloud, disparityMap)
    %% ============= Parameters ============= %%
%     THETA_X_DEG = 12;
%     THETA_X_RAD = degtorad(THETA_X_DEG);
%     tform_x = affine3d(makehgtform('xrotate', THETA_X_RAD));
% 
%     THETA_Z_DEG = 5;
%     THETA_Z_RAD = degtorad(THETA_Z_DEG);
%     tform_z = affine3d(makehgtform('zrotate', THETA_Z_RAD));

    % ptCloud = pctransform(ptCloud, tform_x);
    % ptCloud = pctransform(ptCloud, tform_z);
    midPlane = planeModel([1, 0, 0, 0]);
    leftPlane = planeModel([1, 0, 0, 0.6]);
    rightPlane = planeModel([1, 0, 0, -0.6]);
    groundPlane = planeModel([0 -1 0 0.4]);
    frontPlane = planeModel([0 0 1 -1.6]);
    topPlane = planeModel([0 1 0 -0.1]);

    vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
    horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.

    NUM_CONTOURS = 6;
    NUM_CONTOUR_LVL = 2;

    OBS_PADDING = 0.4; % meters in front and behind the obstacle that we care about.
    ROI_SAMPLES = 500; % How many points to use for active plane estimation?
    ROI_X = 1;
    ROI_Y = 2;
    %% ========================================= %%
    
    bestPoly = LargestContours(frameLeftGray, NUM_CONTOUR_LVL, NUM_CONTOURS);
    
    [HEIGHT, WIDTH] = size(disparityMap);
    % Set rtnCloud to false because we dont care about the cloud any more.
    [fullCloud, horizPlanes, vertPlanes, planeList] = PlanarizePointCloud(ptCloud, bestPoly, WIDTH, HEIGHT, false);
    
    % Get boundaries for each plane.
    % Front plane is horizontal, back plane is vertical
    planeList = [frontPlane.Parameters; planeList; topPlane.Parameters];
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
end

