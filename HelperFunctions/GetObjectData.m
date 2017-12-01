function object_data = GetObjectData(point_data)
% We will need to add in the logic to deal with the
% point cloud data and find stair heights here



maxDistance = 0.2;
referenceVector = [0,1,0];
maxAngularDistance = 5;
[model1,inlierIndices,outlierIndices] = pcfitplane(point_data.point_cloud,...
            maxDistance,referenceVector,maxAngularDistance);
plane1 = select(point_data.point_cloud,inlierIndices);
remainPtCloud = select(point_data.point_cloud,outlierIndices);
roi = [-inf,inf;0.4,inf;-inf,inf];
sampleIndices = findPointsInROI(remainPtCloud,roi);
[model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,...
            maxDistance,'SampleIndices',sampleIndices);
plane2 = select(remainPtCloud,inlierIndices);
remainPtCloud = select(remainPtCloud,outlierIndices);

figure(10)
cla
pcshow(plane1)
title('First Plane')
xlabel('x')
ylabel('y')
zlabel('z')
object_data = [];