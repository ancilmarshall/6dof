% Simulate response for a constant tilt command during braking maneuver
% Transitions to the correct angle to maintain a fixed position at the
% end of the maneuver taking into account wind. 

clear all;
close all;

% config
t0 = 0;
tf = 4;
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

% exact simulation
%[tout,yout] = ode45('test_rbody5d_eom',[t0 tf],states);

% objects
rbody = RBody5D(states,dt);
angleInput = AngleInput(dt);

% producer registration
rbody.angleCommandProducer = angleInput;

% set cmds
thetaCmd0 = 40*pi/180;
angleInput.thetaCmd = thetaCmd0;

% add Wind

setappdata(0,'env_wind_vwx',vwx);
setappdata(0,'env_wind_vwy',vwy);

% paramters
%angleInput.rateLimitCorrection = 25 *pi/180;

% optimization
%angleInput.vxTransition = 6.1;

% sim
while (rbody.time < tf) 
   angleInput.step;
   rbody.step;
end

% write
rbody.write;
angleInput.write;

% % responses
figure;
%subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(rbody_time,angleInput_thetaCmd*180/pi,'r--');
if (i_compare_exact); plotg(tout,yout(:,3)*180/pi,'m--'); end;
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
if (i_compare_exact); plotg(tout,yout(:,2),'m--'); end;
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
title('Accel X response');
ylabel('Accel X (m/s^2)');
% subplot(212);
% plotg(rbody_time,rbody_ay);
% title('Accel Y response');
% ylabel('Accel Y (m/s^2)');

figure;
% subplot(211)
plotg(rbody_time,rbody_x);
title('Distance X response');
ylabel('Distance X (m)');
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
plotg(rbody_time,angleInput_thetaCmd*180/pi,'r--');
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
ylim([0 30]);


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