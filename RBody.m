classdef RBody < handle
   
    properties
        
        % get from mass producer
        inertia = eye(3);
        
        integrator;
        dt;
        time;
        states = [];      
        writer; % writer saves time history of states in arrays so that
                % this object does not have to. The writer class can be 
                % generic and used by several objects
        
        prefix = 'rbody';
        stateLabels = {...
            'omega_x',...
            'omega_y',...
            'omega_z'};
    end

    methods
        
        function self = RBody(states,dt)
            self.time = 0; % TODO - generalize initial time
            self.states = states;
            self.dt = dt;
            
            self.integrator = Integrator(self.states,self.dt);
            self.writer = Writer(self.prefix,self.stateLabels);
            
            % write the first set of data. Could argue ghat this can be
            % done upon construction. 
            self.writer.updateTime(self.time);
            self.writer.updateStates(self.states);
        end
        
        function registerIntegrator(self,integrator)
            self.integrator = integrator;
        end
        
        function step(self)
            
            % get all derivates updates from all the producers
            omega = self.states';
            I = self.inertia;
            self.integrator.derivatives = (inv(I)*(-cross(omega,I*omega) - omega))';
  
            % perform the update integration step
            [self.time,self.states] = self.integrator.integrate();
            
            %update the writer
            self.writer.updateTime(self.time);
            self.writer.updateStates(self.states)
            
        end
        
        function write(self)
            self.writer.write();
        end
        
    end
    
end
