% Makes a visualizable polytope given a set of planes.
leftPlane = planeModel([1 0 0 0.4]);
leftNorm = leftPlane.Normal;
leftPt = GetPointOnPlane(leftPlane.Parameters);

rightPlane = planeModel([-1 0 0 0.4]);
rightNorm = rightPlane.Normal;
rightPt = GetPointOnPlane(rightPlane.Parameters);

bottomPlane = planeModel([0 -1 0 0.4]);
backPlane = planeModel([0 0 -1 3]);
frontPlane = planeModel([0 0 1 -1]);

planeList = allPlanes;

% Get boundaries for each plane.
planeList = [frontPlane.Parameters; planeList; backPlane.Parameters];
[apRow, apCol] = size(planeList);

% For each plane, make a polygon using the plane before and after it.
poly3d = zeros(apRow-2, 4, 3); % each polygon has 5 XYZ points

for ii = 2:apRow-1
    befPlane = planeList(ii-1,:);
    plane = planeList(ii,:);
    aftPlane = planeList(ii+1,:);
    
    N1 = plane(1:3);
    A1 = GetPointOnPlane(plane);
    
    % Line caused by intersection with plane before
    N_bef = befPlane(1:3);
    A_bef = GetPointOnPlane(befPlane);
    [bPt, bVect, check] = PlaneIntersect(N1, A1, N_bef, A_bef);
    lPt_bef = LinePlaneIntersect(bVect, bPt, leftPlane.Parameters);
    rPt_bef = LinePlaneIntersect(bVect, bPt, rightPlane.Parameters);

    % Line caused by intersection with plane after
    N_aft = aftPlane(1:3);
    A_aft = GetPointOnPlane(aftPlane);
    [aPt, aVect, check] = PlaneIntersect(N1, A1, N_aft, A_aft);
    lPt_aft = LinePlaneIntersect(aVect, aPt, leftPlane.Parameters);
    rPt_aft = LinePlaneIntersect(aVect, aPt, rightPlane.Parameters);
    
    % Now we have 4 points to define a polygon with.
    poly3d(ii-1,:,:) = [lPt_bef; rPt_bef; rPt_aft; lPt_aft];
end

rotate3d on;
[numPoly, pts, doub] = size(poly3d);
for ii = 1:numPoly
    poly = poly3d(ii,:,:);
    X = poly(:,:,1);
    Y = - 1 * poly(:,:,2);
    Z = poly(:,:,3)
    fill3(X,Y,Z, [0 0 1]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on;
    view([0 -0.2 -1])
end