classdef IProducer < handle
    
    properties
        consumers = [];
        name;
    end
    
    properties (Access = private)
        states;
    end
    
    methods
        function registerConsumer(self,consumer)
            
            if isempty(self.consumers)
                self.consumers = consumer;
            else
                if ~ self.contains(consumer)
                    self.consumers(end+1) = consumer;
                end
            end
            
            % register the reverse relationship 
            % TODO: check first
            
            
        end
        
        function res = contains(self,consumer)
            res = false;
            for i=1:length(self.consumers)
                if strcmp(consumer.name,self.consumers(i).name)
                    res = true;
                    break;
                end
            end
        end
        
    end
end