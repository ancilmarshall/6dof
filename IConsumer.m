classdef IConsumer < handle
    
    properties
        states;
    end
    
    methods
        
        %constructor
%         function self = IConsumer
%             self = IConsumer;
%         end
        
        function updateStates(self,states)
            self.states = states;
        end
        
    end
end