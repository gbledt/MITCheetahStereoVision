function [polygons] = LargestContours(frame, cntLevels, K)
    [C, h] = imcontour(frame, cntLevels);
    cMatrix = h.ContourMatrix;
    shapeData = contour2shape(cMatrix);

    polyAreas = zeros(numel(shapeData), 1);
    for si = 1:numel(shapeData)
        shape = shapeData(si);
        polyAreas(si) = polyarea(shape.X, shape.Y);
    end
    
    [b, ix] = sort(polyAreas(:), 'descend');
    [topIndices, cc] = ind2sub( size(polyAreas), ix(1:K) );
    polygons = shapeData(topIndices);
end

