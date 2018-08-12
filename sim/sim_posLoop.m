% Test acceleration loop with closed loop controller
clear all;
close all;

% config
t0 = 0;
tf = 15;
dt = 0.005;

Cx = 0.35;  % s-1
G = 9.81;   % m/s^2
wn = 11;    %rad/s
zeta = 0.9; % -
Cy = Cx;

% x, vx, theta, q, y, vy, phi, p
v0 = 0;
theta0 = 0*pi/180;
states = [0 v0 theta0 0 0 0 0 0]';

setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

% objects
rbody = RBody5D(states,dt);
positionLoop = PositionLoop(dt);
positionInput = PositionInput(dt);

% producer registration
positionLoop.positionInput = positionInput;
rbody.angleCommandProducer = positionLoop;

%initialize sim
positionInput.xInput = 1;

% sim and event
while rbody.time < tf
   
   %handle events here
   
   if rbody.time > 4
      positionInput.xInput = 1.2;
   end
   
   
   % run sim
   
   positionInput.step;
   positionLoop.step;
   rbody.step;
end

% write
rbody.write;
positionLoop.write;
positionInput.write;

% % response
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(rbody_time,positionLoop_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
plotg(rbody_time,positionLoop_phiCmd*180/pi,'r--');
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
title('Accel X response');
ylabel('Accel X (m/s^2)');
subplot(212);
plotg(rbody_time,rbody_ay);
title('Accel Y response');
ylabel('Accel Y (m/s^2)');

figure;
subplot(211)
plotg(rbody_time,rbody_x);
title('Distance X response');
ylabel('Distance X (m)');
subplot(212);
plotg(rbody_time,rbody_y);
title('Distance Y response');
ylabel('Distance Y (m/s^2)');



