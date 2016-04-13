function [y, delta, startIdx, endIdx] = preprocess(data)
   	
    for joint = 1:size(data,1)
        data(joint,:) = smooth(data(joint,:),11);
    end

    delta = smooth(sqrt(sum(diff(data,1,2).^2,1)),11);
    
    startIdx = -1;
    endIdx = -1;
%     threshold = 1e-4;
    threshold = 1e-3;
    step = 20;
    
    for i = 1:length(delta)-step
        if startIdx < 0 && sum(delta(i:i+step))/(step+1) > threshold
            startIdx = i;
        end
        
        if endIdx < 0 && startIdx > 0 && sum(delta(i:i+step))/(step+1) < threshold
            endIdx = i;
        end   
    end
    
    y = data(:,startIdx:endIdx);
    delta = delta(startIdx:endIdx);
end