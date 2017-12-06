function PlotPoly3D(poly3d)
    figure
    rotate3d on;
    [numPoly, pts, doub] = size(poly3d);
    colors = ['y', 'm', 'c', 'r', 'g', 'b'];
    for ii = 1:numPoly
        poly = poly3d(ii,:,:);
        X = poly(:,:,1);
        Y = poly(:,:,2);
        Z = poly(:,:,3);
        fill3(X,Y,Z, colors(mod(ii, 6)+1));
        hold on;
    end

    % Manipulate figure nicely.
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(3)
    camup([0 -1 0])
    camorbit(120,0,'data',[0 -1 0])
    daspect([1 1 1])
end

