function [w_star,sigma_star] = ProMPs_prediction( X_star, y_star, ws, lambda, DDB)

    [M,N,L] = size(ws);
    S = size(X_star,1) / DDB;
    flgUseRecursiveImpl = 0;

    if size(lambda,1)*size(lambda,2) == 1
        if flgUseRecursiveImpl
            lambdaMat = eye(DDB)* lambda;
        else
            lambdaMat = eye(DDB*S)* lambda;
        end
    else
        if flgUseRecursiveImpl
            lambdaMat = diag(lambda);
        else
            lambdaMat = diag(repmat(lambda,1,S));
        end
    end

    w_star = mean(ws,3);
    sigma_star = cov(squeeze(ws(:,1,:))');


    x_tmp = X_star';
    y_tmp = y_star;  


    sigma_star = sigma_star';
    features = x_tmp';
    tempMatrix = sigma_star * features' / (lambdaMat + features * sigma_star * features');
    newMu = w_star + tempMatrix * (y_tmp - features * w_star);
    newSigma = sigma_star - tempMatrix * features * sigma_star;

    w_star = newMu;
    sigma_star = newSigma;





end

