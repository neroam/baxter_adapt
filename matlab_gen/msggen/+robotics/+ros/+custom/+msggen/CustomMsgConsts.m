classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    properties (Constant)
        baxter_adapt_Adaptation = 'baxter_adapt/Adaptation'
        baxter_adapt_AdaptationRequest = 'baxter_adapt/AdaptationRequest'
        baxter_adapt_AdaptationResponse = 'baxter_adapt/AdaptationResponse'
        baxter_adapt_Learning = 'baxter_adapt/Learning'
        baxter_adapt_LearningRequest = 'baxter_adapt/LearningRequest'
        baxter_adapt_LearningResponse = 'baxter_adapt/LearningResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(6, 1);
                msgList{1} = 'baxter_adapt/Adaptation';
                msgList{2} = 'baxter_adapt/AdaptationRequest';
                msgList{3} = 'baxter_adapt/AdaptationResponse';
                msgList{4} = 'baxter_adapt/Learning';
                msgList{5} = 'baxter_adapt/LearningRequest';
                msgList{6} = 'baxter_adapt/LearningResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(2, 1);
                svcList{1} = 'baxter_adapt/Adaptation';
                svcList{2} = 'baxter_adapt/Learning';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
    end
end
