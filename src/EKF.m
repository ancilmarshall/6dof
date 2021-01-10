classdef EKF < handle
    % Fundamentals of Kalman Filtering by Zarchan
    % Edx: Sensor Fusion and Non-linear Filtering for Automotive Systems
    % Coursera: State-estimation-localization-self-driving-cars/lecture/qIyk3/lesson-3-going-nonlinear-the-extended-kalman-filter
    % Optimal State Estimation. Kalman, Hinf by Dan Simon. Chapter 13

    % TODO: Add the discrete G term (see Zarchan) for the process
    % TODO: Is L and M needed? How does this compare to Zarchan.
    %       Are these for the EKF versions?
    properties
        % callback fun ctions
        f_func  % f(x, u, w) - nonlinear state equation
        h_func  % h(x, v)    - nonlinear model or observation equation

        F_func  % df/dx - state jacobian matrix
        L_func  % df/dw - process noise jacobian matrix (shown in coursera)
        H_func  % dh/dx - model state jacobian matrix
        M_func  % dh/dv - model noise jacobian matrix (shown in coursera)

        Q_func; % discrete process covariance matrix
        R_func; % discrete measurement noise covariance matrix

        % TODO: Change to nx, ny, nu
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
        function self = EKF(xhat, Phat, callbacks)

            % FIXME: Who is responsible for properly initializing the filter?
            self.xhat = xhat;
            self.Phat = Phat;

            % assign callbacks
            self.f_func = callbacks{1}; % ["state_func"];
            self.h_func = callbacks{2}; % ["model_func"];
            self.F_func = callbacks{3}; % ["F_func"];
            self.L_func = callbacks{4}; % ["L_func"];
            self.H_func = callbacks{5}; % ["H_func"];
            self.M_func = callbacks{6}; % ["M_func"];
            self.Q_func = callbacks{7}; % Discrete Noise covariance matrix
            self.R_func = callbacks{8}; % Discrete Meas covariance matrix

            self.n = length(xhat);

        end

        function [stateEst, stateCov] = step(self, dt, in, obs)
            % step the filter on time step
            % dt : time interval since last update
            % in : input u
            % obs: observed data, measurement data vector. Optional input
            %      because I can propagate the equations without the measurement
            %      fusion
            % TODO: user inputParser so that observation is first

            % Check for a new observation
            newObs = false;
            if nargin == 4
                newObs = true;
            end

            self.Q = self.Q_func(self.xhat, dt); % need to pass dt
            self.R = self.R_func();

            % Propagation Step
            F = self.F_func(self.xhat);
            L = self.L_func(self.xhat);
            Phi = eye(self.n) + F*dt;   % state transition. Approximation

            % [TODO] add and integrator here to propagate the state
            % Use simple euler integration to propagate the states
            % third argument is noise value which is zero for zero mean
            %   xdot = f(x,u,w), w ~N(0,Q)
            self.xbar = self.xhat + self.f_func(self.xhat, in, zeros(self.n,1)) * dt;
            self.Pbar = Phi * self.Phat * Phi' + L * self.Q * L';

            % Update step
            if (newObs)
                H = self.H_func(self.xhat); % TODO: xbar or xhat?
                M = self.M_func(self.xhat); % TODO: xbar or xhat?

                self.K = self.Pbar * H' * pinv(H * self.Pbar * H' + M * self.R * M');

                % filter residual or innovation
                innovation = obs - self.h_func(self.xbar, 0);

                self.xhat = self.xbar + self.K * innovation;
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
