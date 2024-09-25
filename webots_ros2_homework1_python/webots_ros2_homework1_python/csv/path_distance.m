function d = dist(point1x, point1y, point2x, point2y)
    d = sqrt(power((point2x - point1x), 2) + power((point2y - point1y), 2));
end

function p = path_dist(xcol, ycol)
    sum = 0;
    for i = 1:length(xcol) - 1
        sum = sum + dist(xcol(i), ycol(i), xcol(i + 1), ycol(i + 1));
    end
    p = sum;
end

Array = readtable("G:\Programming\CS 460\f24_robotics\webots_ros2_homework1_python\webots_ros2_homework1_python\csv\area1\trial6.csv");
col1 = Array{:, 1};
col2 = Array{:, 2};
disp(path_dist(col1, col2));
plot(col1, col2);
axis([-20 20 -20 20]);