classdef (Abstract) IConsumer < handle
    
    properties (Access = private)
        producers = [];
    end
    
    properties 
        states;
    end
    
    methods
        
        %constructor
%         function self = IConsumer
%             self = IConsumer;
%         end
        function registerProducer(self,producer)
            % check if producers is not already in the list
            % not sure about retain cycle
            % check the type
            
            if isempty(self.producers)
                self.producers = producer;
            else 
                self.producers(end+1) = producer;
            end
            
            % register the reverse relationship
            % TODO: check first
            producer.registerConsumer(self);
            
        end
        
        

        
    end
end