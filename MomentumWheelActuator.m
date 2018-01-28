classdef MomentumWheelActuator < handle
    % assume 3 actuators aligned with the body x y z axis
    
    properties
        
        inertia = 0.01*eye(3);
        
        integrator;
        dt;
        time;
        states = [];      % or data vs inegrator states
        %[Omegax,OmegaDotx,Omegay,OmegaDoty,Omegaz,OmegaDotz]
        
        initialStates = [];
        
        control = []; % control input. OmegaxCmd OmegayCmd OmegazCmd
        
        forces = [];
        moments = [];
        
        writer; % writer saves time history of states in arrays so that
                % this object does not have to. The writer class can be 
                % generic and used by several objects
    end
    
    properties (Access = private )
        numStates;
        name = 'act';
        outputVars = {
            'Omegax'
            'OmegaDotx'
            'Omegay'
            'OmegaDoty'
            'Omegaz'
            'OmegaDotz'
            };
        
        % configuration data
        zeta = 0.707; % (critically damped)
        wn = 25/(2*pi); % 15Hz actuator bandwidth
    end
    
    methods        
        
        % constructor
        function self = MomentumWheelActuator(states,dt)
            self.time = 0; % TODO - generalize initial time
            self.states = states;
            self.dt = dt;
            
            %initial contitions
            
            
            %%% fixme - the number of integration states vs input data
            self.integrator = Integrator(self.states,self.dt);
            self.writer = Writer(self.name,self.outputVars);
            
            % write the first set of data. Could argue ghat this can be
            % done upon construction. 
            self.writer.updateTime(self.time);
            
            self.writer.updateData(self.states);
            
            self.numStates = length(states);
        end
        
        function step(self)
            
            w = self.states(1:3);
            
            % TODO rewrite so that I don't use this block, or rewrite
            % using blocks but keeping original state order
            % Omegax,Omegay,Omegaz
            A = zeros(6,6);
            B = zeros(6,3);
            for axis = 1:3
                blockSize = 2;
                row = axis*blockSize - 1;
                col = row;
                Ablock = [0 1;-self.wn^2 -2*self.zeta*self.wn];
                Bblock = [0;self.wn^2];
                A(row:row+blockSize-1,col:col+blockSize-1) = Ablock;
                B(row:row+blockSize-1,axis) = Bblock;
            end
           
            x = self.states;
            u = [self.control];
            self.integrator.updateDerivatives(A*x+B*u);
               
            % perform the update integration step
            [self.time,self.states] = self.integrator.step();

            %update the writer
            self.writer.updateTime(self.time);
            self.writer.updateData(self.states);
            
%             for i=1:length(self.consumers)
%                 consumer = self.consumers(i);
%                 consumer.updateStates(self.states)
%             end
            
        end
        
        function write(self)
            self.writer.write;
        end
        
        function updateForces(self)
            self.forces = zeros(3,1);
        end;
        
        function updateMoments(self)
            w = self.states(1:3);
            Omega = self.states(4:6);
            OmegaDot = self.states(7:9);
            I = self.inertia;
            self.moments = - ( I*OmegaDot + cross(w,I*Omega) );
        end
        
        
        %%%  Force producer interface
        function forces = getForces(~)
            forces = self.forces;
        end
        
        function moments = getMoments(~)
            moments = self.moments;
        end
        
    end
    
    
end