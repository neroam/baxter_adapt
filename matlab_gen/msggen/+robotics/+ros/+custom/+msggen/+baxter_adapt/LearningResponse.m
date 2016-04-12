classdef LearningResponse < robotics.ros.Message
    %LearningResponse MATLAB implementation of baxter_adapt/LearningResponse
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'baxter_adapt/LearningResponse' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'b5a4974dd6799144216b3d605aef404f' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Response
        Filename
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Filename', 'Response'} % List of non-constant message properties
        ROSPropertyList = {'filename', 'response'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = LearningResponse(msg)
            %LearningResponse Construct the message object LearningResponse
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function response = get.Response(obj)
            %get.Response Get the value for property Response
            response = logical(obj.JavaMessage.getResponse);
        end
        
        function set.Response(obj, response)
            %set.Response Set the value for property Response
            validateattributes(response, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'LearningResponse', 'response');
            
            obj.JavaMessage.setResponse(response);
        end
        
        function filename = get.Filename(obj)
            %get.Filename Get the value for property Filename
            filename = char(obj.JavaMessage.getFilename);
        end
        
        function set.Filename(obj, filename)
            %set.Filename Set the value for property Filename
            validateattributes(filename, {'char'}, {}, 'LearningResponse', 'filename');
            
            obj.JavaMessage.setFilename(filename);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Response = obj.Response;
            cpObj.Filename = obj.Filename;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Response = strObj.Response;
            obj.Filename = strObj.Filename;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Response = obj.Response;
            strObj.Filename = obj.Filename;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.baxter_adapt.LearningResponse.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.baxter_adapt.LearningResponse;
            obj.reload(strObj);
        end
    end
end
