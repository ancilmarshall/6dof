classdef EKF < handle
    % Optimal State Estimation. Kalman, Hinf by Dan Simon. Chapter 13
    % https://www.coursera.org/learn/state-estimation-localization-self-driving-cars/lecture/qIyk3/lesson-3-going-nonlinear-the-extended-kalman-filter
    % https://en.wikipedia.org/wiki/Extended_Kalman_filter
    properties
        % callback functions
        f_func  % f(x, u, w) - nonlinear state equation
        h_func  % h(x, v)    - nonlinear model or observation equation
        
        F_func  % df/dx - state jacobian matrix
        L_func  % df/dw - process noise jacobian matrix (shown in coursera)
        H_func  % dh/dx - model state jacobian matrix
        M_func  % dh/dv - model noise jacobian matrix (shown in coursera)
        
        %
        n       % number of states
        m       % number of outputs
        
        Q       % process noise matrix
        R       % measurement noise matrix
        
        % A priori estimate (prediction)
        xbar
        Pbar
        
        % A posteriori estimate (update)
        xhat
        Phat
        
        % Kalman gain
        K
        
    end
    
    methods
        
        % constructor
        function self = EKF(xhat, Phat, Q, R, callbacks)
                        
            % FIXME: Who is responsible for properly initializing the filter?
            self.xhat = xhat;
            self.Phat = Phat;
            self.Q = Q;
            self.R = R;
            
            % assign callbacks
            self.f_func = callbacks{1}; % ["state_func"];
            self.h_func = callbacks{2}; % ["model_func"];
            self.F_func = callbacks{3}; % ["F_func"];
            self.L_func = callbacks{4}; % ["L_func"];
            self.H_func = callbacks{5}; % ["H_func"];
            self.M_func = callbacks{6}; % ["M_func"];
            
            self.n = length(xhat);
            
            
            
        end
        
        function [stateEst, stateCov] = step(self, dt, in, obs)
            
            % Check for a new observation
            newObs = false;
            if nargin == 4
                newObs = true;
            end
            
            % Propagation Step
            F = self.F_func(self.xhat);
            L = self.L_func(self.xhat);
            
            % [TODO] add and integrator here to propagate the state
            % Use simple euler integration to propagate the states
            self.xbar = self.xhat + self.f_func(self.xhat, in, 0) * dt;
            self.Pbar = F * self.Phat * F' + L * self.Q * L';
            
            % Update step
            if (newObs)
                H = self.H_func(self.xbar);
                M = self.M_func(self.xbar);
                
                self.K = self.Pbar * H' * pinv(H * self.Pbar * H' + M * self.R * M');
                self.xhat = self.xbar + self.K * (obs - self.h_func(self.xbar, 0));
                self.Phat = (eye(self.n) - self.K * H) * self.Pbar;
            else
                self.xhat = self.xbar;
                self.Phat = self.Pbar;
            end
            % return data
            stateEst = self.xhat;
            stateCov = self.Phat;
        end
        
    end
    
    
end
