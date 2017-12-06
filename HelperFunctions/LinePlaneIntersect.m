function [xyz_point, status] = LinePlaneIntersect(line_direction, line_pt, plane_params)
    %% Parameters %%
    % INPUT
    % line_direction: A 3 element vector
    % line_pt: A 3 element xyz point
    % plane_params: [a, b, c, d]
    
    % OUTPUT
    % xyz_point: The point of intersection if one exists, else infinity
    % stats: 0 = found point, 1 = line on plane, 2 = no intersection
    %% =============== %%

    num = -1 * (dot(plane_params(1:3), line_pt) + plane_params(4));
    denom = dot(plane_params(1:3), line_direction);
    
    % If denom is zero, then the line is parallel to the plane
    status = 0;
    if abs(denom) < 0.0001
        % Case 1: num is also zero, line is on the plane.
        if abs(num) < 0.0001
            disp("Line is on the plane");
            status = 1;
            xyz_point = line_pt;
        else
            disp("Line is parallel to plane, no intersection");
            status = 2;
            xyz_point = [inf, inf, inf];
        end
    else
        T = num / denom;
        xyz_point = line_pt + T * line_direction;
    end
end

