function [ y_improved ] = getFeedback( y )
% get feedback trajectory from mouse input and smooth it.
    
[T, dim] = size(y);

hFh = imfreehand();
feedback = hFh.getPosition;
delete(hFh);

[~, startIdx] = min(sum((repmat(feedback(1,:), T,1) - y).^2, 2));
[~, endIdx] = min(sum((repmat(feedback(end,:), T,1) - y).^2, 2));

for i = 1:dim
    feedback(:,dim) = smooth(feedback(:,dim));
end

% figure;
% plot(feedback(:,1), feedback(:,2));
% 
% step = floor((size(feedback,1) - 2) / (endIdx - startIdx - 1));
% y_improved = y;
% j = 1+min(step,1);
% 
% for i = startIdx+1:endIdx-1
%     y_improved(i,:) = feedback(j,:);
%     j = j+step;
% end

y_improved = y;
y_improved(startIdx:endIdx,:) = feedback(round(linspace(1,size(feedback,1),endIdx-startIdx+1)),:);


slack = 5;
for i = 1:dim
    y_improved(max(startIdx - slack, 1): min(endIdx + slack, T),dim) = smooth(y_improved(max(startIdx - slack, 1): min(endIdx + slack, T),dim));
end

% figure;
% plot(y_improved(:,1), y_improved(:,2));

end

