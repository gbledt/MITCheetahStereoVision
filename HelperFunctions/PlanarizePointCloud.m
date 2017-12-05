function [fullCloud, horizPlanes, vertPlanes] = PlanarizePointCloud(ptCloud, bestPoly, WIDTH, HEIGHT)
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

    M = zeros(2, numel(bestPoly));
    for ii = 1:numel(bestPoly)
        % Fit a plane to each segmented region of the scene.
        poly = bestPoly(ii);
        poly.X = [poly.X, poly.X(1)];
        poly.Y = [poly.Y, poly.Y(1)];
        bw = poly2mask(poly.X, poly.Y, HEIGHT, WIDTH);
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
    end

end

