% Perform cross track guidance with a Velocity Loop
%clear all;
close all;

global Cx Cy G wn zeta thetaCmd phiCmd

rtd = 180/pi;

% config
t0 = 0;
tf = 7;
dt = 0.005;

Cx = 0.35;  % s-1
G = 9.81;   % m/s^2
wn = 11;     %rad/s
zeta = 0.9; % -
Cy = Cx;

thetaCmd = -30*pi/180;
phiCmd = 0*pi/180;
% x v theta q y w phi r]
states = [0 1 0 0 0 0 0 0]';

% exact simulation
%[tout,yout] = ode45('test_rbody5d_eom',[t0 tf],states);

setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

setappdata(0,'data_guidance_userThetaCmd',thetaCmd);
setappdata(0,'logic_guidance_state',1); % activate guidance loop

setappdata(0,'env_wind_vwx',0);
setappdata(0,'env_wind_vwy',0);

% objects
rbody = RBody5D(states,dt);
accelLoop = AccelLoop(dt);
guidance = AccelGuidanceLoopTrajRef(dt);

% producer registration
rbody.angleCommandProducer = accelLoop; % velocityLoop
accelLoop.accelCommandProducer = guidance;

% need to set this to properly init the guidance loop
waypointManager = WaypointManager;
waypointManager.add(20,0,0,false); %false = not safe, must stop
% waypointManager.add(Waypoint(10,5,0)); %false = not safe, must stop
% waypointManager.add(Waypoint(20,0,0)); %false = not safe, must stop
% waypointManager.add(Waypoint(30,0,0)); %false = not safe, must stop
% waypointManager.add(Waypoint(40,10,0,false)); %false = not safe, must stop

guidance.setWaypointManager(waypointManager);

% sim
while rbody.time < tf
   rbody.step;
   guidance.step;
   accelLoop.step;
end

% write
rbody.write;
accelLoop.write;
guidance.write;

% responses
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
%plotg(rbody_time,accelLoop_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
%plotg(rbody_time,accelLoop_phiCmd*180/pi,'r--');
ylabel('Phi (deg)');
title('Body Phi');
xlabel('Time (sec)');

% 
figure;
subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
title('Velocity X response');
subplot(212);
plotg(rbody_time,rbody_vy);
ylabel('vy (m/s)');
title('Velocity Y response');


% cross track error and distance to go error
figure;
subplot(211);
plotg(guidance_time,guidance_distToGoError);
hold on;
idx = find(guidance_state==1,1,'first');
%plotg(guidance_time(idx),guidance_distCritical(idx),'m--');
% thres = guidance_distCritical(idx);
% yline(thres);
title('Distance to go');
ylabel('Distance (m)');
xlabel('Time (sec)');

subplot(212);
plotg(guidance_time,guidance_crossTrackError);
title('Cross Track Error');
xlabel('Time (sec)');
ylabel('Cross Track Error (m)');



figure;
subplot(211)
plotg(rbody_time,rbody_ax);
hold on;
%plotg(guidance_time,guidance_axCmd,'r--');
title('Accel X response');
ylabel('Accel X (m/s^2)');
subplot(212);
plotg(rbody_time,rbody_ay);
title('Accel Y response');
ylabel('Accel Y (m/s^2)');


figure;
plotg(guidance_time,guidance_state);
title('Accel Loop Guidance State. 0-Flying, 1-Braking, 2-Levelout');
ylabel('State (-)');
xlabel('Time (sec)');
ylim([0 1.2]);

figure;
plotg(guidance_time,guidance_xRef,'r--');
hold on;
plotg(rbody_time,rbody_x,'b');
title('X ref');
ylabel('Distance (m)');
xlabel('Time (s)');


if ~isempty(idx)
   
   ii = find(guidance_state >= 1);
   time = rbody_time(ii) - rbody_time(ii(1));
   ax = rbody_ax(ii);
   dist = guidance_distToGoError(ii);
   vx = rbody_vx(ii);
   theta = rbody_theta(ii)*180/pi;
   
   
   figure;
   subplot(221)
   plotg(time,ax);
   ylabel('Accel (m/s^2)');
   title('Performance during braking to fixed waypoint');
   
   subplot(222);
   plotg(time,vx);
   ylabel('Velocity (m/s)');
   
   subplot(223);
   plotg(time,dist);
   ylabel('Dist To Go (m)');
  
   subplot(224);
   plotg(time,theta);
   ylabel('Theta (deg)');
   xlabel('Time during braking (sec)');
   
end


% accel loop controller
figure
plotg(axLoopPID_time,axLoopPID_control*rtd);
hold on;
plotg(axLoopPID_time,axLoopPID_controlProp*rtd,'r');
plotg(axLoopPID_time,axLoopPID_controlInt*rtd,'g');
plotg(axLoopPID_time,axLoopPID_controlDeriv*rtd,'c');
plotg(axLoopPID_time,axLoopPID_controlFF*rtd,'k--');
legend('total','prop','int','deriv','ff');
xlabel('Time (sec)');
ylabel('Theta command (deg)');
title('Accleration Loop Control Performance');

%link;


% ground track
% figure;
% plotg(rbody_y,rbody_x);
% title('Ground track');
% axis equal
% hold on;
% wpts = waypointManager.wptArray;
% %plotg(wpts(:,2),wpts(:,1),'k--');
% plotg(wpts(:,2),wpts(:,1),'ro');

%fanfigs;


