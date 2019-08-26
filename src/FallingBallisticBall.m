classdef FallingBallisticBall < handle & IWriter
    
    properties
        
        integrator
        dt
        time
        states = [];
        
        use_measurement_noise = true;  % add noise to sensor output by default
        
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
        
        height_noise = 100;      % 1-sigma height noise value
        vel_noise = 10;          % 1-sigma velocity noise value
        coeff_noise = 0.000125;  % 1-sigma coeff noise value
        
        % paramters
        rho = 0.0034; % [lb-sec^2/ft^4] air density at sea-level
        k = 22000;  % [ft] constant between rho and altitude
        G = 32.2;  % [ft/s^2]
        PhiS = 100.0;
    end
    
    methods
        
        % Constructor
        function self = FallingBallisticBall(initial_states, dt)
            self.time = 0;
            self.states = initial_states;
            self.dt = dt;
            self.updateData();
            
            self.integrator = Integrator(self.states, self.dt);
            self.writer = Writer(self.name, self.outputVars, ...
                @() [self.time;self.states(1);self.states(2);self.states(3)]);
            self.writer.step();
        end
        
        function step(self)
                      
            % Integration of the state
            % No input for these equations xdot = f(x,u,w);  u = 0
            params = [self.rho, self.k, self.G];
            xdot = FallingBallisticBall.state_equations(self.states, 0, 0, params);
            self.integrator.updateDerivatives(xdot);
            [self.time, self.states] = self.integrator.step();
            
            % update data other than states
            self.updateData();
            
            % update the writer
            self.writer.step();
            
            % write output
            self.updateDataManager();
            
        end
        
        function updateData(self)
            % update any internal data after states have been updated
            self.height = self.states(1);
            self.velocity = self.states(2);
            self.coeff = self.states(3);
        end
        
        function out = sensorOutput(self)
            % sensor available is only the measurement of the height
            % update sensed output variables
            
            % noise values
            if self.use_measurement_noise
                h_noise = self.height_noise * randn;
            else
                h_noise = 0;
            end

            out = self.states(1) + h_noise;
            
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
            
            rho0 = params(1);
            k = params(2);
            G = params(3);
            
            rho = rho0 * exp(-x1/k);
            %input
            % No input for these equations xdot = f(x,u,w);  u = 0
            
            % nonlinear equations of motion
            x1dot = x2;
            x2dot = rho * G * x2^2 / ( 2 * x3) - G;
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
            
            rho0 = params(1);
            k = params(2);
            G = params(3);
            
            rho = rho0 * exp(-x1/k);
            A21 = -rho * G * x2^2  / (2 * k * x3);
            A22 = rho * G * x2 / x3;
            A23 = - rho * G * x2^2 / (2 *  x3^2 );
            

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
        
        function Q = process_covar(states, dt, params)
            x1 = states(1);    % height
            x2 = states(2);    % velocity
            x3 = states(3);    % ballistic coefficient
            
            rho0 = params(1);
            k = params(2);
            G = params(3);
            PhiS = params(4);
            
            rho = rho0 * exp(-x1/k);
            
            A23 = - rho * G * x2^2 / (2 *  x3^2 );
            
            Q = PhiS * [ 0      0                0 ;
                         0   A23^2*dt^3/3    A23*dt^2/2; 
                         0    A23*dt^2/2        dt ];            
        end
       

    end
    
end

