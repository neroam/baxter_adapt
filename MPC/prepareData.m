function datasets = prepareData(synthetic)

synthestic_data = synthetic;

if synthestic_data
    %% Create synthetic data
    numDemonstrations = 50;
    DOF = 2;

    datasets = struct([]);

    T = 500;
    
    t=zeros(DOF,T);
    for i=1:DOF
        t(i,:) = linspace(0,i*0.25*pi,T);
    end

    hFigRawData=figure; hold all;
    set(hFigRawData,'Color','white');
    for l=1:numDemonstrations
        data = (1+ 0.3*randn(1,1))*sin(t) + (0.3+ 0.1*randn(1,1))*sin(3*t) + 0.1*randn(DOF,T);
        datasets(l).Y = data(:); %encoding: [val1DoF1, val1DoF2, val1DoF3, ..., val2DoF1, ...]
        datasets(l).Ymat = data; %for plotting only
        plot(data(1,:));
    end
else
    %% Read from real data
    training_folder = '../training/';
    fnames = dir([training_folder 'training*']);
    numDemonstrations = length(fnames);
    datasets = struct([]);
    T = 200;
    
    hFigRawData=figure; hold all;
    set(hFigRawData,'Color','white');
    for l = 1:numDemonstrations
        fname = fnames(l).name;
        data = csvread([training_folder fname], 1, 0);
        datasets(l).Ymat = preprocess(data(:,9:15)');
        datasets(l).Ymat = datasets(l).Ymat(:,round(linspace(1,size(datasets(l).Ymat,2),T)));
        datasets(l).Y = datasets(l).Ymat(:);
        plot(datasets(l).Ymat(1,:));
    end
  
end

xlabel('time step', 'fontsize',26);
ylabel('raw data of joint 1', 'fontsize',26);
set(gca, 'fontsize',26);

end