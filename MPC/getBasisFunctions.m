function phi = getBasisFunctions( numBasisFuns, timesteps,nder)
    % nder = number of derivatives (only 1 or 2 supported)
    [c,h] = createBasisFunParams(numBasisFuns);
    z = linspace(0,1,timesteps);
    phi = zeros(nder, numBasisFuns,timesteps);
    
    % compute stroke based basis functions for each timestep
    b = exp(-1 * (repmat(z,numBasisFuns,1) - repmat(c',1,timesteps)).^2 / (2*h));
    % compute derivatives: phi_dot= phi' * z' (with approximation)
    b_dot = b .* ((repmat(c',1,timesteps) - repmat(z,numBasisFuns,1)) ./ h);
    
    sum_b = repmat(sum(b,1),numBasisFuns,1);
    sum_b_dot= repmat(sum(b_dot,1),numBasisFuns,1);
    % normalize basis functions for each time step
    phi(1,:,:) = b ./ sum_b;
    if nder == 2
        % compute derivatives: phi_dot= phi' * z' (with approximation)
        b_dot = (b_dot .* sum_b - b .* sum_b_dot) ./  (sum_b .^ 2);
        phi(2,:,:) = b_dot * (z(2) - z(1));
    end
    % only for testing---------------------
%     plotBasisFunctions(phi,nder);    
        
end

function [c,h] = createBasisFunParams(numFuns)
    if numFuns > 5
        p = 1/(numFuns-3);
        c = [0-p, linspace(0,1, numFuns-2), 1+p];
    else
        c = linspace(0,1,numFuns); % centers are linear spread between (0,1) 
    end
    if numFuns > 1
        % norm is always the same because of linear centers
        h = 2 *  0.5 * (c(2) - c(1));
        h = h^2; 
    else
        h = 1;
    end
end

%-- plotting basis functions --------------
function plotBasisFunctions(bas_fun,nder)
    for i = 1:nder
       figure
       plot(squeeze(bas_fun(i,:,:))')
    end
end