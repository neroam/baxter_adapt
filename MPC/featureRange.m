function [s, e] = featureRange(y_ref, y, obstacles)

changeRange = find(sum((y_ref - y).^2,2) > 0.1)
obstacleRange = find(sum((y - obstacles).^2, 2) < 0.5)

s = min(intersect(changeRange, obstacleRange))
e = max(intersect(changeRange, obstacleRange))

end