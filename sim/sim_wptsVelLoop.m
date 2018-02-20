% Perform cross track guidance with a Velocity Loop
clear all;
close all;

global Cx Cy G wn zeta thetaCmd phiCmd

% config
t0 = 0;
tf = 20;
dt = 0.005;

Cx = 0.35;  % s-1
G = 9.81;   % m/s^2
wn = 11;     %rad/s
zeta = 0.9; % -
Cy = Cx;

thetaCmd = -5*pi/180;
phiCmd = 0*pi/180;
states = [0 0 0 0 0 0 0 0]';


%%% Configuration data
setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

% objects
rbody = RBody5D(states,dt);
velocityLoop = VelocityLoop(dt);
guidance = VelocityGuidanceLoop(dt);

% producer registration
rbody.angleCommandProducer = velocityLoop;
velocityLoop.guidance = guidance;

% need to set this to properly init the guidance loop
waypointManager = WaypointManager;
waypointManager.add([10 5 0]');
waypointManager.add([20 5 0]');
waypointManager.add([30 0 0]');
waypointManager.add([40 0 0]');

guidance.setWaypointManager(waypointManager);

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

% plots
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
%plotg(rbody_time,velocityLoop_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
%plotg(rbody_time,velocityLoop_phiCmd*180/pi,'r--');
ylabel('Phi (deg)');
title('Body Phi');
xlabel('Time (sec)');


figure;
subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
%plotg(guidance_time,guidance_vxCmd,'r--');
ylabel('vx (m/s)');
title('Velocity X response');
subplot(212);
plotg(rbody_time,rbody_vy);
hold on;
%plotg(guidance_time,guidance_vyCmd,'r--');
ylabel('vy (m/s)');
title('Velocity Y response');


% ground track
figure;
plotg(rbody_y,rbody_x);
title('Ground track');
axis equal
hold on;
wpts = waypointManager.wptArray;
plotg(wpts(:,2),wpts(:,1),'k--');
plotg(wpts(:,2),wpts(:,1),'ro');


% cross track error
figure;
plotg(guidance_time,guidance_crossTrackError);
title('Cross Track Error');
xlabel('Time (sec)');
ylabel('Cross Track Error (m)');

fanfigs;


