classdef (Abstract) IConsumer < handle
    
    properties (Access = private)
        producers = [];
        name;
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
                if ~ self.contains(producer)
                    self.producers(end+1) = producer;
                end
            end
            
            % register the reverse relationship
            % TODO: check first
            producer.registerConsumer(self);
            
        end
        
        function res = contains(self,producer)
            res = false;
            for i=1:length(self.producers)
                if strcmp(producer.name,self.producers(i).name)
                    res = true;
                    break;
                end
            end
        end

        
    end
end