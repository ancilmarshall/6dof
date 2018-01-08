classdef IProducer < handle
    
    properties
        consumers = IConsumer();
    end
    
    methods
        function addConsumer(self,consumer)
            consumers(end+1) = consumer;
        end
    end
end