function check()

global weights;
    
if exist('weights','var') == 0 || length(weights) ~= 10
    weights = [1 1 1 1 1 1 1 1 1 1];
    weights
end

weights = weights*0.1

end