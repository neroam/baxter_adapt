function w = ProMPs_train( datasets, M, N, lambdaProMPs)
    L = size(datasets,2);
    w = zeros (M,N,L);
    for i = 1:L
        X_train = datasets(i).PSI_X;
        y_train = datasets(i).Y;
        w(:,:,i) = (X_train' * X_train + eye(M)*lambdaProMPs) \ X_train' * y_train;
    end

end



