% test of the 5D equations of motion in NED ( z assumed constant ) 
clear;
close all;

global Cx Cy G wn zeta thetaCmd phiCmd

% config
t0 = 0;
tf = 10;
dt = 0.005;

Cx = 0.35;  % s-1
G = 9.81;   % m/s^2
wn = 11;     %rad/s
zeta = 0.9; % -
Cy = Cx;

thetaCmd = -5*pi/180;
phiCmd = 0*pi/180;
states = [0 0 0 0 0 0 0 0]';

% exact simulation
%[tout,yout] = ode45('test_rbody5d_eom',[t0 tf],states);

setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

setappdata(0,'config_control_kp',0.2612);
setappdata(0,'config_control_ki',0.1606);
setappdata(0,'config_control_kd',0);
setappdata(0,'data_control_controlFF',0);

wpt = [10 5 0]';
setappdata(0,'data_guidance_wpt',wpt);

% objects
rbody = RBody5D(states,dt);
velocityLoop = VelocityLoop(dt);
guidance = VelocityGuidanceLoop(dt);

% producer registration
rbody.angleCommandProducer = velocityLoop;
velocityLoop.guidance = guidance;


% sim
while rbody.time < tf
   rbody.step;
   velocityLoop.step;
   guidance.step;
end

% write
rbody.write;
velocityLoop.write;
guidance.write;

% response
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(rbody_time,velocityLoop_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
plotg(rbody_time,velocityLoop_phiCmd*180/pi,'r--');
ylabel('Phi (deg)');
title('Body Phi');
xlabel('Time (sec)');


figure;
subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
plotg(guidance_time,guidance_vxCmd,'r--');
ylabel('vx (m/s)');
title('Velocity X response');
subplot(212);
plotg(rbody_time,rbody_vy);
hold on;
plotg(guidance_time,guidance_vyCmd,'r--');
ylabel('vy (m/s)');
title('Velocity Y response');


% ground track
figure;
plotg(rbody_y,rbody_x);
title('Ground track');
axis equal
hold on;
plot(wpt(2),wpt(1),'ro');
plot([0 wpt(2)],[0 wpt(1)],'k--');

% cross track error
figure;
plotg(guidance_time,guidance_crossTrackError);
title('Cross Track Error');
xlabel('Time (sec)');
ylabel('Cross Track Error (m)');

fanfigs;


