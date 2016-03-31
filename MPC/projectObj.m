function [f, g] = projectObj(U, W)

f = (U-W)'*(U-W);
g = 2*(U-W);

end