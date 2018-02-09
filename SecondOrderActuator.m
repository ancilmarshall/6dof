classdef SecondOrderActuator < handle
    % assume 3 actuators aligned with the body x y z axis
    
    properties
        
        inertia = 0.1;
        mass = 0.1;
        
        integrator;
        dt;
        time;
        states = [];      % Omega, OmegaDot
                
        control = 0; % control input. OmegaxCmd OmegayCmd OmegazCmd
        
        forces = 0;
        moments = 0;
        
        controller;
        
        writer; % writer saves time history of states in arrays so that
                % this object does not have to. The writer class can be 
                % generic and used by several objects
    end
    
    properties (Access = private )
        numStates;
        name = 'act';
        
        %%% inputs
        
        %%% outputs
        
        
        %%% log variables
        outputVars = {
            'Omega'
            'OmegaDot'
            };
        
        %%% configuration data
        zeta; % damping ratio
        wn;   % natural frequency 
    end
    
    methods        
        
        % constructor
        function self = SecondOrderActuator(states,dt)
            self.time = 0; % TODO - generalize initial time
            self.states = states;
            self.dt = dt;
            
            %%%configuration
            
            self.wn = getappdata(0,'config_act_wn');
            self.zeta = getappdata(0,'config_act_zeta');
            
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
            
            %%% input
            self.control = self.controller.getCommand;
%             p = getappdata(0,'data_rbody_p');
%             q = getappdata(0,'data_rbody_q');
%             r = getappdata(0,'data_rbody_r');
            
            % the control loop here
            %self.control = self.w;
            A = [0 1;-self.wn^2 -2*self.zeta*self.wn];
            B = [0;self.wn^2];
           
            
            x = self.states;
            u = self.control;
            self.integrator.updateDerivatives(A*x+B*u);
               
            % perform the update integration step
            [self.time,self.states] = self.integrator.step;
            
            % perform state calculations with new data
            self.updateForces;
            self.updateMoments;

            %update the writer
            self.writer.updateTime(self.time);
            self.writer.updateData(self.states);
            
%             for i=1:length(self.consumers)
%                 consumer = self.consumers(i);
%                 consumer.updateStates(self.states)
%             end

            %%% output
            setappdata(0,'data_act_Omega',self.states(1));
            setappdata(0,'data_act_OmegaDot',self.states(2));
            
        end
        
        function write(self)
            self.writer.write;
        end
        
        function updateForces(self)
            self.forces = zeros(3,1);
        end
        
        function updateMoments(self)
            OmegaDot = self.states(2);
            I = self.inertia;
            self.moments = - I*OmegaDot; % not sure a out the minus sign
        end
        
        %%%  Force producer interface
        function forces = getForces(self)
            forces = self.forces;
        end
        
        function moments = getMoments(self)
            moments = self.moments;
        end
        
        %%% Mass producer interface
        function mass = getMass(self)
            mass = self.mass;
        end
        
        function inertia = getInertia(self)
            inertia = self.inertia;
        end
        
    end
    
    
end