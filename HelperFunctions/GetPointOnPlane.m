function xyz_point = GetPointOnPlane(plane_params)
    origin = [0 0 0];
    dist = dot(origin, plane_params(1:3)) + plane_params(4);
    xyz_point = origin - dist * plane_params(1:3);
end

