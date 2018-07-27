% Simulate response when given an angle input command which is zero'd
% in order that vehicle comes to a stop. 
clear all;
close all;

% config
t0 = 0;
tf = 6;
dt = 0.005;
i_compare_exact = 0;

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
v0 = 30.0;
theta0 = -30*pi/180;
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
angleInput.thetaCmd = 30*pi/180;

% add Wind
rbody.vwx = 0;
rbody.vwy = 0;
angleInput.vwx = 0;
angleInput.vwy = 0;

% sim
while (rbody.time < tf) && ( rbody.states(2) > -0.025 ) 
   rbody.step;
   angleInput.step;
end

% write
rbody.write;
angleInput.write;

% % response
figure;
subplot(211);
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(rbody_time,angleInput_thetaCmd*180/pi,'r--');
if (i_compare_exact); plotg(tout,yout(:,3)*180/pi,'m--'); end;
ylabel('Theta (deg)');
title('Body Theta');
subplot(212);
plotg(rbody_time,rbody_phi*180/pi);
hold on;
plotg(rbody_time,angleInput_phiCmd*180/pi,'r--');
if (i_compare_exact); plotg(tout,yout(:,7),'m--'); end;
ylabel('Phi (deg)');
title('Body Phi');
xlabel('Time (sec)');


figure;
subplot(211);
plotg(rbody_time,rbody_vx);
hold on;
if (i_compare_exact); plotg(tout,yout(:,2),'m--'); end;
title('Velocity X response');
subplot(212);
plotg(rbody_time,rbody_vy);
hold on;
if (i_compare_exact); plotg(tout,yout(:,6),'m--'); end;
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



