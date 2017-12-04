% [bw, xi, yi] = roipoly(frameLeftGray);
% [ptCloudSmooth, plane_model] = SmoothMaskedSurface(bw, ptCloud);

% [ptCloudSmooth, out] = removeInvalidPoints(ptCloud);
fullCloud = pointCloud([0,0,0]);
for ii = 1:5
    [bw, xi, yi] = roipoly(frameLeftGray);
    smoothSurface = SmoothMaskedSurface(bw, ptCloud);
    fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);
end

player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y','VerticalAxisDir', 'down');
view(player3D, fullCloud);
% hold on;
% plot(plane_model);

function [plane_model, ptCloudSeg] = fitPlane(mask, ptCloud)
    indices = find(mask);
    ptCloudSeg = select(ptCloud, indices);
    [plane_model,inlierIndices,outlierIndices] = pcfitplane(ptCloudSeg, 0.03);
end