classdef Writer < handle
    
    properties
       time;
       states;
       stateLabels;
       prefix;
    end
    
    properties (Access = private)
        numStates
    end
    
    methods
        
        % TODO: initial with initial time,states
        function self = Writer(prefix,stateLabels)
            self.prefix = prefix;
            self.stateLabels = stateLabels;
            self.numStates = length(stateLabels);
        end
        
        function updateTime(self,time)
            if isempty(time)
                self.time = time;
            else
                self.time(end+1,1) = time;
            end
        end
        
        function updateStates(self,states)
            if isempty(states)
                self.states = states;
            else
                self.states(end+1,:) = states;
            end
            
        end
        
        function write(self)
            assignin('base',[self.prefix '_time'],self.time);
            for i=1:self.numStates
                label = [self.prefix '_' self.stateLabels{i}];
                state = self.states(:,i);
                assignin('base',label,state);
            end
        end
        
    end
    
end