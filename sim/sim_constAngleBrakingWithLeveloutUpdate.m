% Simulate response for a constant tilt command during braking maneuver
% Transitions to the correct angle to maintain a fixed position at the
% end of the maneuver taking into account wind. 

clear all;
close all;

% config
t0 = 0;
tf = 15;
dt = 0.005;
i_compare_exact = 0;
i_print = 0;

% Bebop
% Cx = 0.35;  % s-1
% G = 9.81;   % m/s^2
% wn = 11;    %rad/s
% zeta = 0.9; % -
% Cy = Cx;

% anafi
Cx = 0.47;  % s-1
G = 9.81;   % m/s^2
wn = 11;    %rad/s
zeta = 0.9; % -
Cy = Cx;

% initial conditions
% x, vx, theta, q, y, vy, phi, p

vwx = 0;
vwy = 0;

vair = 15;

v0 = vair+vwx;
theta0 = -40*pi/180;
states = [0 v0 theta0 0 0 0 0 0]';

setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

% objects
rbody = RBody5D(states,dt);
angleInput = AngleInput(dt);
positionLoop = PositionLoop(dt);
positionInput = PositionInput(dt);
positionRef = PositionRefLQServo(dt);

% producer registration
rbody.angleCommandProducer = angleInput;
positionRef.positionInput = positionInput;
positionLoop.positionRef = positionRef;


% set cmds
thetaCmd0 = 20*pi/180;

% set gains
% design 1 - overshoot in theta
% positionRef.Kp = 7.73;
% positionRef.Kd = 9.3;
% positionRef.Ka = 5.35;
% positionRef.Ki = -3.16;
% positionRef.alpha = 0.87;

% design 2 - Q = diag([1 100 100]);
% good. slower response in v, and acc i.e. less overshoot.
% slow response on x
positionRef.Kp = 10.921;
positionRef.Kd = 18.7;
positionRef.Ka = 11.722;
positionRef.Ki = -3.16;
positionRef.alpha = .87; % it's strange that this has big influence anot
 %  part of the LQR output. Not robust at all


% add Wind

setappdata(0,'env_wind_vwx',vwx);
setappdata(0,'env_wind_vwy',vwy);

% paramters
%angleInput.rateLimitCorrection = 25 *pi/180;

% optimization
%angleInput.vxTransition = 6.1;

% initialize sim
angleInput.thetaCmd = thetaCmd0;
phase = 'flying'; %flying, braking, levelout
while (rbody.time < tf) 
   
   %handle events here
   if rbody.time > 1
      phase = 'braking';
   end
   
   velTransition = angleInput.getLeveloutTransitionVx;
   if strcmp(phase,'braking') && (angleInput.vx < 2)
      positionInput.xInput = 19; 
      rbody.angleCommandProducer = positionLoop;
      
      positionInput.activate;
      positionRef.activate;
      positionLoop.activate;
      
      positionInput.step;
      positionRef.step;
      positionLoop.step;
   else
      angleInput.step
   end
   
   rbody.step;
end

% write
rbody.write;
angleInput.write;
positionInput.write;
positionRef.write;
positionLoop.write;

% % responses
figure;
%subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(angleInput_time,angleInput_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title('Body Theta');
% subplot(212);
% plotg(rbody_time,rbody_phi*180/pi);
% hold on;
% plotg(rbody_time,angleInput_phiCmd*180/pi,'r--');
% if (i_compare_exact); plotg(tout,yout(:,7),'m--'); end;
% ylabel('Phi (deg)');
% title('Body Phi');
% xlabel('Time (sec)');

figure;
% subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
title('Velocity X response');
% subplot(212);
% plotg(rbody_time,rbody_vy);
% hold on;
% if (i_compare_exact); plotg(tout,yout(:,6),'m--'); end;
% ylabel('vy (m/s)');
% title('Velocity Y response');

figure;
% subplot(211)
plotg(rbody_time,rbody_ax);
hold on;
plotg(positionRef_time,positionRef_ax,'r--');
hold on;
title('Accel X response');
ylabel('Accel X (m/s^2)');
% subplot(212);
% plotg(rbody_time,rbody_ay);
% title('Accel Y response');
% ylabel('Accel Y (m/s^2)');

figure;
% subplot(211)
plotg(rbody_time,rbody_x);
hold on;
plotg(positionRef_time,positionRef_x,'k--');
plotg(positionInput_time,positionInput_xInput,'r--');
title('Rbody X response');
ylabel('Rbody X (m)');
% subplot(212);
% plotg(rbody_time,rbody_y);
% title('Distance Y response');
% ylabel('Distance Y (m/s^2)');

% figure;
% subplot(211)
% plotg(rbody_time,rbody_q);
% title('Pitch Rate response');
% ylabel('Pitch Rate (rad/s)');
% subplot(212);
% plotg(rbody_time,rbody_p);
% title('Roll Rate response');
% ylabel('Roll Rate (rad/s)');
% xlabel('Time (sec)');


figure;
subplot(311)
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(angleInput_time,angleInput_thetaCmd*180/pi,'r--');
ylabel('Theta (deg)');
title({'{\bf \fontsize{14} Braking At Constant Theta With Wind}';...
       ['C_x = ', num2str(Cx) ' s^{-1}, Initial Vgnd = ' ...
            num2str(v0) ' m/s, theta Cmd = ' sprintf('%3.1f',thetaCmd0*180/pi) ' deg, ' ...
            'Wind ' num2str(vwx) ' m/s']});
ylim([-45 50]);
xlim([0 tf]);
         
subplot(312)
plotg(rbody_time,rbody_vx);
title('Ground Speed');
ylabel('Ground Speed (m/s)');
xlim([0 tf]);
ylim([-5 20]);


subplot(313)
plotg(rbody_time,rbody_x);
title('Distance');
ylabel('Distance(m)');
xlabel('Time (sec)');
xlim([0 tf]);
%ylim([0 60]);


if i_print
if vwx > 0
   print(gcf,'-dpng',...
      sprintf('stoppingDistanceTrajs45_tailwind_%d',vwx));
else
   print(gcf,'-dpng',...
      sprintf('stoppingDistanceTrajs45_headwind_%d',-vwx));
end
end
disp(max(rbody_x))
fanfigs