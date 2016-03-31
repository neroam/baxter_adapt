function [g] = featureObstacle(y_improved, y, obstacles, w)

[T, ~] = size(y);
obstacles = repmat(obstacles', T, 1);
distance_improved = sum((y_improved-obstacles).^2,1);
distance_orig = sum((y-obstacles).^2,1);
g = distance_improved*exp(-w*distance_improved') - distance_orig*exp(-w*distance_orig');

end