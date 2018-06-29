classdef Integrator < handle
    
    properties
       dt;
       time;
       states = [];
    end
    
    properties (Access = private)
        producers = [];
        consumers = [];
        derivatives = [];
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
        
        function [tout,xout] = step(self)
            %check if states is non-empty
            %check that derivatives are set
            
            % simple euler integration
            % TODO: improve to RK-4 integration
            self.states = self.states + self.derivatives * self.dt;
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
        

        

        
        

        
        
    end
    
    
end
