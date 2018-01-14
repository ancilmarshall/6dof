classdef IProducer < handle
    
    properties 
        consumers = [];
    end
    
    properties (Access = private)
        states;
    end
    
    methods
        function registerConsumer(self,consumer)
            
            if isempty(self.consumers)
                self.consumers = consumer;
            else
                self.consumers(end+1) = consumer;
            end
            
            % register the reverse relationship 
            % TODO: check first
            
        end
    end
end