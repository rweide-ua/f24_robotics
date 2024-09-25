function d = dist(point1x, point1y, point2x, point2y)
    d = sqrt(pow2(point2x - point1x) + pow2(point2y - point1y));
end

dist(0, 0, 1, 1);