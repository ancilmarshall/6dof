% Simulate the EKF using the FallingBallisticBall model from:
% Optimal State Estimation. Kalman, Hinf by Dan Simon. Chapter 13
% Example 13.2

clear;
close all;

% config
dt = 1/50;  % 100 Hz
tf = 3;      % [sec] - simulation end time

%inital data
height = 100000; % [m]
velocity = -6000; % [m/s]
coeff = 1/2000;    % [ ? ] Ballistic coefficient
initial_states = [height; velocity; coeff];

% objects
model = FallingBallisticBall(initial_states, dt);

% extended kalman filter setup
% callback functions for the nonlinear problem
% use a mix of the static class member with instance parameters to define
% the instance specific parameters, and keeping the interface of the
% callbacks simple and in agreement with the textbook equations
% Note could use static parameters, or parameters configure in an external
% file, but uses the instance parameters for flexibility
params = [model.rho, model.k, model.G];
state_equ = @(x, u, w) FallingBallisticBall.state_equations(x, u, w, params);
measurement_equ = @(x, v) FallingBallisticBall.sensor_equations(x, v);

% jacobians
F_equ = @(x) FallingBallisticBall.state_jacobian(x, params);
L_equ = @(x) FallingBallisticBall.process_jacobian();
H_equ = @(x) FallingBallisticBall.model_jacobian();
M_equ = @(x) FallingBallisticBall.measurement_jacobian();

callbacks = {
   state_equ
   measurement_equ
   F_equ
   L_equ
   H_equ
   M_equ
   };

P0 = [500    0      0;
       0   20000    0;
       0     0  1/250000];
Q = diag([0, 0, 0]);
R = 100;

ekf = EKF(initial_states, P0, Q, R, callbacks);
% connect objects

t = [0:dt:tf]';
m = length(t);         % number of observations
n = length(initial_states);     % number of states
xEst = zeros(m,n);
PEst = zeros(n,n,m); 

% initialize the output
i = 1;
xEst(1,:) = initial_states';
PEst(:,:,1) = P0;

% sim
while (model.time < tf)
   i = i+1;
   model.step();
   in = 0; % control input u to the state equations
   obs = model.height;
   %obs = model.sensor_equations(model.states, 0); % zero sensor noise
   [x, p] = ekf.step(dt, in, obs);
   xEst(i,:) = x';
   PEst(:,:,i) = p;
end

% write output to matlab
model.write();
% ekf.write(); [TODO] Make a writer class

estimate_height = xEst(:,1);
estimate_velocity = xEst(:,2);
estimate_coeff = xEst(:,3);
estimate_time = t;

figure(1);
hold on;
error_height = estimate_height - ballisticBall_height;
error_time = t;
plotg(error_time,error_height);

