% test the extended kalman filter

% Courseara State Estimation and Localisation class example
% Wk 2, Lesson 3 - Going Nonlinear - The Extended Kalman Filter
% states: p, pdot   ( position, velocities)
% input : u = pddot ( acceleration, i.e. pddot)
% output: theta     ( angle )
%
% Noise densities
% w ~ N([0;0], 0.1*eye(2))   - process noise
% v ~ N(0, 0.01)             - measurement noise

clear;

dt = 0.5;   % [sec] step time of the simulation
S = 10;     % [m] Height of reference
D = 100;    % [m] Horizontal distance of reference

% callback functions for the nonlinear problem
state_equ = @(x, u, w) [1 dt; 0 1] * x + [0; dt] * u + w;
measurement_equ = @(x, v) atan(S/D-x(1)) + v;

% jacobians
F_equ = @(x) [1 dt;0 1];
L_equ = @(x)  eye(2);
H_equ = @(x) [S/((D-x(1))^2 + S^2), 0];
M_equ = @(x) 1;

callbacks = {
   state_equ
   measurement_equ
   F_equ
   L_equ
   H_equ
   M_equ
   };

% initial conditions

x0 = 0;
x0_sigma = 0.1; 

v0 = 5;
v0_sigma = 1;

% control u
a0 = -2; % m/s2

xEst0 = [x0;v0];
P0 = diag([x0_sigma^2,v0_sigma^2]); % state cov
% remember that variance is the standard deviation squared

%Process Noise - Tuning the filter. Need Q matrix to avoid smug filter
xnoise = 0.1;   % m, 1-sigma values
vnoise = 0.1; % m/s, 1-sigma values

Q = diag([xnoise^2, vnoise^2]);

% Measurment Model
yObsNoise = 0.05;
R = yObsNoise^2;

    
% initialize filter class with the initial state and state cov matrix
kf = EKF(xEst0, P0, Q, R, callbacks);

% data
r = [30 30];
u = [a0 a0];
m = length(r);         % number of observations
n = length(xEst0);     % number of states

xEst = zeros(m,n);
PEst = zeros(n,n,m); 

% initialze output
xEst(1,:) = xEst0';
PEst(:,:,1) = P0;

% run filter from the 2nd data point through m
for i=2:m
   [x, p] = kf.step(r(i),u);
   xEst(i,:) = x';
   PEst(:,:,i) = p;
end
   


