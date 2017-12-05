function shp = contour2shape(cmatrix)
    %Converts a contour matrix to a shape structure
    %needs https://www.mathworks.com/matlabcentral/fileexchange/43162-c2xyz-contour-matrix-to-coordinates
    [x, y, z] = C2xyz(cmatrix);
    shp = struct('Geometry', 'PolyLine', 'X', x, 'Y', y, 'Z', num2cell(z));
end