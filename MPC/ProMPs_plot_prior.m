function [hFigModel, y_predProMPs] = ProMPs_plot_prior(datasets, wsProMPs, lambdaProMPs, flgPlotData, DDB)

    wsProMPsPrior = mean(wsProMPs,3);
    sigmaProMPs = cov(squeeze(wsProMPs(:,1,:))');
    y_predProMPs = datasets(1).PSI_X * wsProMPsPrior;
    y_predVarProMPs = zeros(size(y_predProMPs));
    for t=1:size(y_predProMPs,1)
        y_predVarProMPs(t,:) =  datasets(1).PSI_X(t,:)*sigmaProMPs*datasets(1).PSI_X(t,:)' + lambdaProMPs;
    end
    
    T = size(y_predProMPs,1) / DDB; % for each task different number of timesteps
    y_predProMPs = reshape(y_predProMPs,DDB,T)';
    y_predVarProMPs = reshape(y_predVarProMPs,DDB,T)';
    
    timeIds = linspace(0,1,T);
    
         
    lw = 2;
    fs = 26;
    fsLegend=18;
    L = size(datasets,2);
    cMap = jet(L);
    hFigModel = zeros(1,DDB);
    
    for i=1:DDB   
        
         hFigModel(i) = figure;
         set(hFigModel(i),'Color','white');
         hold all;
     
        lndPred =  shadedErrorBar(timeIds,y_predProMPs(:,i),sqrt(y_predVarProMPs(:,i)),'r',1);
        est_plot = lndPred.mainLine;
        set(est_plot, 'linewidth', lw);
   
    
    
     if flgPlotData > 0
        
         tmpData = []; 
         for trajId=1:L
            Tdata = size(datasets(trajId).Y,1) / DDB; 
            y_true = reshape(datasets(trajId).Y,DDB,Tdata)';
            timeIds = linspace(0,1,length(y_true(:,i)));

            plot(timeIds, y_true(:,i), 'linewidth', 1, 'color', 'b');
            
         end
         

     end
    m = datasets(1).PSI_X*wsProMPsPrior;
    m = reshape(m,DDB,T)';
    timeIds = linspace(0,1,T);
    plot(timeIds, m(:,i),'k', 'linewidth', 2*lw);
    
    %axis tight

    
    xlabel('movement phase', 'fontsize', fs);
    ylabel(['prior of joint ' num2str(i)], 'fontsize', fs);
    set(gca, 'fontsize', fs);
 
    hold off;
    end
end