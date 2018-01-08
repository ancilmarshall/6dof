classdef Integrator < handle
    
    properties
       dt;
       time;
       states = [];
       derivatives = [];
       producers = [];
       consumers = [];
       
    end
    
    methods
        
        % constructor
        function self = Integrator(states,dt)
            self.time = 0;
            self.states = states;
            self.dt = dt;
        end
        
        function activate(~)
            
        end
        
        function updateDerivatives(self,derivatives)
            self.derivatives = derivatives;
        end   
        
        function [tout,xout] = integrate(self)
            %check if states is non-empty
            %check that derivatives are set
            
            % simple euler integration
            % improve to RK-4 integration
            self.states = self.states + self.derivatives .* self.dt;
            for i=1:length(self.consumers)
                self.consumers(i).updateStates(self.states)
            end
            
            self.time = self.time + self.dt;
            
            tout = self.time;
            xout = self.states;
        end
        
        function setStates(self,states)
            self.states = states;
        end
        
        function output = getStates(self)
            output = self.states;
        end
        
        function register(self)
            
        end
        
        function unregister(self)
            
        end
        
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
        end
        
        function registerConsumer(self,consumer)
            
            if isempty(self.consumers)
                self.consumers = consumer;
            else
                self.consumers(end+1) = consumer;
            end
            
            % register the reverse relationship
        end
        
        

        
        
    end
    
    
end
