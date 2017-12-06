function [fullCloud, horizPlanes, vertPlanes, allPlanes] = PlanarizePointCloud(ptCloud, bestPoly, WIDTH, HEIGHT)
    % INPUT:
    % ptCloud: a pointCloud object
    % bestPoly: a list of polygon regions in image to fit planes to
    % WIDTH: width of the depthmap
    % HEIGHT: height of the depthmap
    
    % OUTPUT
    % fullCloud: a point cloud where planar regions have been fit
    % horizPlanes: planes that are close to horizontal
    % vertPlanes: planes that are close to vertical
    % allPlanes: all planes from supplied regions
    
    %% ======== Params =======%%
    orientationThresh = 1.5;
    
    % Note: this assumes RDF frame convention.
    vertNorm = [0 0 -1]; % Unit vector to compare vertical planes against.
    horizNorm = [0 1 0]; % Unit vector to compare horiz planes against.
    %% ======================= %%
    
%     [HEIGHT, WIDTH] = size(disparityMap);
    fullCloud = pointCloud([0,0,0]);

    % Detect and store all horizontal planes.
    horizPlanes = [];

    % Detect and store all horizontal planes.
    vertPlanes = [];
    
    % Store all planes here
    allPlanes = [];
    
    % Sort polygons by increasing height in the original image (useful
    % later)
    numPoly = numel(bestPoly);
    centroids = zeros(numPoly, 2);
    for pp = 1:numPoly
        poly = bestPoly(pp);
        polyXY = [transpose(poly.X), transpose(poly.Y)];
        centroids(pp,:) = Centroid2D(polyXY);
    end
    [b, ix] = sort(centroids(:,2), 'descend');
%     bestPoly = bestPoly(ix,:)

    for ii = 1:numPoly
        % Fit a plane to each segmented region of the scene.
        poly = bestPoly(ix(ii));
        poly.X = [poly.X, poly.X(1)];
        poly.Y = [poly.Y, poly.Y(1)];
        bw = poly2mask(poly.X, poly.Y, HEIGHT, WIDTH);
%         figure, imshow(bw)
        [smoothSurface, plane_model] = SmoothMaskedSurface(bw, ptCloud);
        fullCloud = pcmerge(fullCloud, smoothSurface, 0.001);

        % Score planes based on their dot product against references unit
        % vectors.
        horizScore = abs(dot(plane_model.Normal, horizNorm));
        vertScore = abs(dot(plane_model.Normal, vertNorm));

        % Check if this is a horizontal plane.
        if (horizScore / vertScore) > orientationThresh
            horizPlanes = [horizPlanes; plane_model.Parameters];
        end

        % Also check if a vertical plane.
        if (vertScore / horizScore) > orientationThresh
            vertPlanes = [vertPlanes; plane_model.Parameters];
        end
        
        allPlanes = [allPlanes; plane_model.Parameters];
    end

end

