% Identify acceleration tau based on the accel loop closed-loop response
clear all;
close all;
global tau input
tau = 0.2;
input = -6;

% config
t0 = 0;
tf = 1.5;
dt = 0.005;

Cx = 0.35;  % s-1
G = 9.81;   % m/s^2
wn = 11;    %rad/s
zeta = 0.9; % -
Cy = Cx;

userThetaCmd = 20*pi/180;
userPhiCmd = 0*pi/180;

% x, vx, theta, q, y, vy, phi, p
v0 = 5;
theta0 = -20*pi/180;
states = [0 v0 theta0 0 0 0 0 0]';

setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);
setappdata(0,'data_guidance_userThetaCmd',userThetaCmd);
setappdata(0,'data_guidance_userPhiCmd',userPhiCmd);

setappdata(0,'logic_guidance_state',1); % 0 - open loop theta, 1 - close accel loop

% exact simulation
[tout,yout] = ode45('tau_eom',[t0 tf],1.821);

% objects
rbody = RBody5D(states,dt);
accelLoop = AccelLoop(dt);
guidance = AccelGuidanceLoop(dt);

% producer registration
rbody.angleCommandProducer = accelLoop;
accelLoop.guidance = guidance;

% set the guidance command
guidance.axCmd = -6;
guidance.ayCmd = 0;

% sim
while rbody.time < tf
   rbody.step;
   accelLoop.step;
   %guidance.step;
end

% write
rbody.write;
accelLoop.write;
%guidance.write;

% % response
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(rbody_time,accelLoop_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
plotg(rbody_time,accelLoop_phiCmd*180/pi,'r--');
ylabel('Phi (deg)');
title('Body Phi');
xlabel('Time (sec)');


figure;
subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
title('Velocity X response');
subplot(212);
plotg(rbody_time,rbody_vy);
hold on;
ylabel('vy (m/s)');
title('Velocity Y response');

figure;
subplot(211)
plotg(rbody_time,rbody_ax);
hold on;
plotg(tout,yout,'r--');
title('Accel X response');
ylabel('Accel X (m/s^2)');
subplot(212);
plotg(rbody_time,rbody_ay);
title('Accel Y response');
ylabel('Accel Y (m/s^2)');


