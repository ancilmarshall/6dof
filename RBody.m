classdef RBody < IProducer
    properties
        
        % get from mass producer
        inertia = eye(3);
        mass = 1;
        
        integrator;
        dt;
        time;
        states = [];      
        writer; % writer saves time history of states in arrays so that
                % this object does not have to. The writer class can be 
                % generic and used by several objects
        
        act; % for testing the actuator producer. 
    end
    
    properties (Access = private )
        numStates;
        name = 'rbody';
        outputVars = {
            'u'
            'v'
            'w'
            'omega_x'
            'omega_y'
            'omega_z'
            'q0'
            'q1'
            'q2'
            'q3'
            };
    end

    methods
        
        function self = RBody(states,dt)
            self.time = 0; % TODO - generalize initial time
            self.states = states;
            self.dt = dt;
            
            self.integrator = Integrator(self.states,self.dt);
            self.writer = Writer(self.name,self.outputVars);
            
            % write the first set of data. Could argue ghat this can be
            % done upon construction. 
            self.writer.updateTime(self.time);
            self.writer.updateData(self.states);
            
            self.numStates = length(states);
        end
        
        function registerIntegrator(self,integrator)
            self.integrator = integrator;
        end
        function step(self)
            
            v = self.states(1:3);
            omega = self.states(4:6);
            q = self.states(7:10);
            
            % get all forces and moments from all the producers, and update
            % the integrator derivative
            
            forces = self.act.getForces;
            moments = self.act.getMoments;
            
            I = self.inertia + self.act.getInertia;
            m = self.mass + self.act.getMass;
            
            % set the integrator derivatives for all states
            
            vdot = -(1/m)*(cross(omega,v) + forces);
            omegadot = -inv(I)*(cross(omega,I*omega) + moments);
            
            qskew = [ 0 -omega(3) omega(2) ; omega(3) 0 -omega(1) ; -omega(2) omega(1) 0 ];
            qdot = 0.5 * [ 0 -omega'; omega -qskew ]*q;

            self.integrator.updateDerivatives([vdot;omegadot;qdot]);
               
            % perform the update integration step
            [self.time,self.states] = self.integrator.step();
            
            %update the writer
            self.writer.updateTime(self.time);
            self.writer.updateData(self.states)
            
            for i=1:length(self.consumers)
                consumer = self.consumers(i);
                consumer.updateStates(self.states)
            end
            
            setappdata(0,'data_rbody_p',omega(1));
            setappdata(0,'data_rbody_q',omega(2));
            setappdata(0,'data_rbody_r',omega(3));
            
        end
        
        function write(self)
            self.writer.write();
        end
        
    end
    
end
