% Extracts the best contours from image and smooths.

bestPoly = LargestContours(frameLeftGray, 2, 10);
plot(bestPoly(1).X, bestPoly(1).Y);
hold on;
for ii = 2:8
    plot(bestPoly(ii).X, bestPoly(ii).Y);
end

[HEIGHT, WIDTH] = size(disparityMap);
fullCloud = pointCloud([0,0,0]);
for ii = 1:numel(bestPoly)
    poly = bestPoly(ii);
    poly.X = [poly.X, poly.X(1)];
    poly.Y = [poly.Y, poly.Y(1)];
    bw = poly2mask(poly.X, poly.Y, HEIGHT, WIDTH);
    smoothSurface = SmoothMaskedSurface(bw, ptCloud);
    fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);
end

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');
view(player3D, fullCloud);