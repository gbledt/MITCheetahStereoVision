function [polygons] = LargestContours(frame, cntLevels, K)
    % Get contours from the image.
    [C, h] = imcontour(frame, cntLevels);
    cMatrix = h.ContourMatrix;
    shapeData = contour2shape(cMatrix);
    
    % Get height and width of the input image.
    [HEIGHT, WIDTH] = size(frame);

    % Get area of each contour.
    polyAreas = zeros(numel(shapeData), 1);
    for si = 1:numel(shapeData)
        shape = shapeData(si);
        polyAreas(si) = polyarea(shape.X, shape.Y);
    end
    
    [b, ix] = sort(polyAreas(:), 'descend');

    indices = zeros(1, K); % This will contain final indices of the best contours.
    fill_idx = 1; % Index in indices that we are currently trying to fill.
    ii = 1; % Index in shapeData that we are currently testing.
    
    while (fill_idx <= K && ii < numel(ix))
        
        % Get candidate and make sure it's connected.
        candidate = shapeData(ix(ii));
        candidate.X = [candidate.X, candidate.X(1)];
        candidate.Y = [candidate.Y, candidate.Y(1)];
        
        % Compare candidate against contours to skip redundant ones.
        for jj = 1:(fill_idx-1)
            other = shapeData(indices(jj));
            other.X = [other.X, other.X(1)];
            other.Y = [other.Y, other.Y(1)];
            
            cand_mask = poly2mask(candidate.X, candidate.Y, HEIGHT, WIDTH);
            other_mask = poly2mask(other.X, other.Y, HEIGHT, WIDTH);
            
            area_cand = sum(cand_mask(:)==1);
            intersect = cand_mask & other_mask;
            union = cand_mask | other_mask;
             
            intersect_area = sum(intersect(:)==1);
            union_area = sum(union(:)==1);
            
            % If large IOU, these are redundant contours.
            if (intersect_area / union_area) > 0.2
                ii = ii + 1;
                disp('Breaking out');
                break   
            end
        end
        disp('No strong intersections, adding cand');
        indices(fill_idx) = ix(ii);
        fill_idx = fill_idx + 1;
        ii = ii + 1;
    end
    % Finally, use indices to grab the best polygons.
    polygons = shapeData(indices);
    
end

