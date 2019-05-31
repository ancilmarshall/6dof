classdef FallingBallisticBall < handle & IWriter
    
    properties
        
        integrator
        dt
        time
        states = [];
        
        name = 'ballisticBall';
        outputVars = {
            'height'
            'velocity'
            'coeff'
            };
        
        % states
        height      % height of object
        velocity    % velocity of object
        coeff       % 1/ballistic coefficient that is constant
        
        % paramters
        rho = 0.0034; % [lb-sec^2/ft^4] air density at sea-level
        k = 22000;  % [ft] constant between rho and altitude
        G = 32.2;  % [ft/s^2]
    end
    
    methods
        
        % Constructor
        function self = FallingBallisticBall(initial_states, dt)
            self.time = 0;
            self.states = initial_states;
            self.dt = dt;
            self.updateOutput();
            
            self.integrator = Integrator(self.states, self.dt);
            self.writer = Writer(self.name, self.outputVars, ...
                @() [self.time;self.height;self.velocity;self.coeff]);
            self.writer.step();
        end
        
        function step(self)
            
            % integration step
            %          xdot = FallingBallisticBall.state_equations(self.states, 0, 0, ...
            %             [self.rho, self.k, self.G]);
            
            x1 = self.states(1);
            x2 = self.states(2);
            x3 = self.states(3);
            
            %input
            % No input for these equations xdot = f(x,u,w);  u = 0
            
            % nonlinear equations of motion
            x1dot = x2;
            x2dot = -self.rho * exp(-x1/self.k) * x2^2 * x3/2 - self.G;
            x3dot = 0;
            xdot = [x1dot; x2dot; x3dot];
            
            self.integrator.updateDerivatives(xdot);
            [self.time, self.states] = self.integrator.step();
            
            % update internal variables and calculated output
            self.updateOutput();
            
            % update the writer
            self.writer.step();
            
            % write output
            self.updateDataManager();
            
        end
        
        function updateOutput(self)
            % update internal variables
            self.height = self.states(1);
            self.velocity = self.states(2);
            self.coeff = self.states(3);
        end
        
        function updateDataManager(~)
            % do not set this for this project.
            % directly connect each module with each other
            % to communicate data variables
            % consumer/producer method
        end
    end
    
    %%% Static Methods that can be used as callbacks for other classes
    %%% Here it will be tested with the Extended Kalman Filter class
    methods (Static)
        
        function xdot = state_equations(states, input, w, params)
            % TODO: Add process noise w
            x1 = states(1);
            x2 = states(2);
            x3 = states(3);
            
            rho = params(1);
            k = params(2);
            G = params(3);
            
            %input
            % No input for these equations xdot = f(x,u,w);  u = 0
            
            % nonlinear equations of motion
            x1dot = x2;
            x2dot = -rho * exp(-x1/k) * x2^2 * x3/2 - G;
            x3dot = 0;
            xdot = [x1dot; x2dot; x3dot];
            
        end
        
        function out = sensor_equations(states, v)
            % TODO: Add measurement noise v
            out = states(1);
        end
        
        function F = state_jacobian(states, params)
            % df/dx = F
            x1 = states(1);
            x2 = states(2);
            x3 = states(3);
            rho = params(1);
            k = params(2);
            
            A21 = rho * exp(-x1/k) * x2^2 * x3 / (2 * k);
            A22 = -rho * exp(-x1/k) * x2 * x3;
            A23 = -rho * exp(-x1/k) * x2^2 * x3 / 2;
            F = [ 0   1   0 ;
                A21 A22 A23;
                0   0   0 ];
        end
        
        function L = process_jacobian()
            % df/dw = L
            L = eye(3);
        end
        
        function H = model_jacobian()
            % dh/dx = H
            H = [1 0 0];
        end
        
        function M = measurement_jacobian()
            % dh/dv = M
            M = 1;
        end
    end
    
end

