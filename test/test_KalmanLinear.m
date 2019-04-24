% test the linear kalman filter example of Vallado eg 10.6
clear;

% initial conditions
x0 = 600;
x0_sigma = 15;

v0 = -45;
v0_sigma = 5;

a0 = 1.5;
a0_sigma = 1;

xEst0 = [x0;v0;a0];
P0 = diag([x0_sigma^2,v0_sigma^2,a0_sigma^2]); % state cov
% remember that variance is the standard deviation squared

%Process Noise - Tuning the filter. Need Q matrix to avoid smug filter
xnoise = 1;   % m
vnoise = 0.1; % m/s
anoise = 0.1; % m/s2

Q = diag([xnoise^2, vnoise^2, anoise^2]);

% Measurment Model
H = [1 0 0]; % only one state is available by the sensors
xObsNoise = 5;
R = xObsNoise^2;

% dynamic model. State transition matrix Phi
dt = 5; % 5 second time interval
Phi = [1 dt 0.5*dt^2;
       0  1 dt;
       0  0  1];
    
% initialize filter class with the initial state and state cov matrix
kf = KF(Phi,H,Q,R,xEst0,P0);

% data
r = [600 385 234 85 2];
m = length(r);         % number of observations
t = [0:dt:20];  
n = length(xEst0);     % number of states

xEst = zeros(m,n);
PEst = zeros(n,n,m); 

% initialze output
xEst(1,:) = xEst0';
PEst(:,:,1) = P0;

% run filter from the 2nd data point through m
for i=2:m
   [x, p] = kf.step(r(i));
   xEst(i,:) = x';
   PEst(:,:,i) = p;
end
   


