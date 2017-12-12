ptCloud = point_data.point_cloud;

maxDistance = 0.02;
vRef = [0, 0, 1];
hRef = [0, -1, 0];
maxAngularDistance = 10;

NUM_VERTICAL = 3;
NUM_HORIZONTAL = 3;

remainPtCloud = ptCloud;

inlierCloud = pointCloud([0 0 0]);

vertModels = [];
for ii = 1:NUM_VERTICAL
    [model, inlierInd, outlierInd] = pcfitplane(remainPtCloud, maxDistance, vRef, maxAngularDistance);
    planeSeg = select(remainPtCloud, inlierInd);
    remainPtCloud = select(remainPtCloud, outlierInd);
    vertModels = [vertModels; model.Parameters];
    inlierCloud = pcmerge(inlierCloud, planeSeg, 0.0001);
end

horizModels = [];
for ii = 1:NUM_HORIZONTAL
    [model, inlierInd, outlierInd] = pcfitplane(remainPtCloud, maxDistance, hRef, maxAngularDistance);
    planeSeg = select(remainPtCloud, inlierInd);
    remainPtCloud = select(remainPtCloud, outlierInd);
    horizModels = [horizModels; model.Parameters];
    inlierCloud = pcmerge(inlierCloud, planeSeg, 0.0001);
end

