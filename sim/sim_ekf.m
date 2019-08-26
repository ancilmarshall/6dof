% Simulate the EKF using the FallingBallisticBall model from:
% Optimal State Estimation. Kalman, Hinf by Dan Simon. Chapter 13
% Example 13.2

clear;
close all;

rng('default');   % reset the random noise generator to have same sequence

% config
dt = 1/100;  % 100 Hz
tf = 5;      % [sec] - simulation end time

%inital data
height = 200000; % [m]
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
;.l,

% noise covariance matrix
Qparams = [model.rho, model.k, model.G, model.PhiS];
Q_equ = @(x,dt) FallingBallisticBall.process_covar(x,dt, Qparams);
R_equ = @() (model.height_noise^2);

callbacks = {
   state_equ
   measurement_equ
   F_equ
   L_equ
   H_equ
   M_equ
   Q_equ
   R_equ
   };

P0 = [500    0      0;
       0   20000    0;
       0     0  1/250000];
Q = diag([300, 100, (0.000125)^2]);
R = (model.height_noise)^2;

% initialize the kalman filter state estimates
dh = 100;
dv = 140;
dcoeff = 0.25*coeff;
xhat_initial = initial_states + [dh; dv; dcoeff];
xhat_initial(1) = 100500;
ekf = EKF(xhat_initial, P0, callbacks);

% connect objects

t = [0:dt:tf]';
m = length(t);         % number of observations
n = length(initial_states);     % number of states
xEst = zeros(m,n);
PEst = zeros(n,n,m);
Sig = zeros(m,n);
K = zeros(m,n);

% initialize the output matrices
% [TODO] Change the output to be only when the filter runs
i = 1;
xEst(1,:) = xhat_initial';
PEst(:,:,1) = P0;
Sig(1,:) = (sqrt(diag(P0)))';
K(1,:) = zeros(1,3);


% sim
epsi  = 1e-6;
while (model.time + epsi < tf)
   i = i+1;
   model.step();
   in = 0; % control input u to the state equations
   obs = model.sensorOutput(); % model sensor output with noise
   [x, p] = ekf.step(dt, in, obs);
   xEst(i,:) = x';
   PEst(:,:,i) = p;
   Sig(i,:) = (sqrt(diag(p)))';
   K(i,:) = ekf.K';
end

% write output to matlab
model.write();
% ekf.write(); [TODO] Make a writer class

estimate_height = xEst(:,1);
estimate_velocity = xEst(:,2);
estimate_coeff = xEst(:,3);
estimate_time = t;

error_time = t;
error_height = estimate_height - ballisticBall_height;
error_velocity = estimate_velocity - ballisticBall_velocity;
error_coeff = estimate_coeff - ballisticBall_coeff;

% Plot Estimate Errors
figure(1);
subplot(311)
hold on;
plotg(error_time,error_height);
plotg(error_time,[Sig(:,1) -Sig(:,1)], 'r--');
title('Height Error');

subplot(312)
hold on;
plotg(error_time,error_velocity);
plotg(error_time,[Sig(:,2) -Sig(:,2)],'r--');
title('Velocity Error');

subplot(313)
hold on;
plotg(error_time, error_coeff);
plotg(error_time, [Sig(:,3) -Sig(:,3)], 'r--');
title('Coeff Error');

% Plot gains
figure(2);
subplot(311)
plotg(t, K(:,1));
title('K height');

subplot(312)
plotg(t, K(:,2));
title('K velocity');

subplot(313)
plotg(t, K(:,3));
title('K coeff');

