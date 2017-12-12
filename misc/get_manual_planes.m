NUM_PLANES = 8;

for i = 1:NUM_PLANES
    [bw, xi, yi] = roipoly(frameLeftGray);
end