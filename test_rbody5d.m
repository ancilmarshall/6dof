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
[tout,yout] = ode45('test_rbody5d_eom',[t0 tf],states);

setappdata(0,'data_control_thetaCmd',thetaCmd);
setappdata(0,'data_control_phiCmd',phiCmd);
setappdata(0,'config_act_wn',wn);
setappdata(0,'config_act_zeta',zeta);
setappdata(0,'config_aero_Cx',Cx);
setappdata(0,'config_aero_Cy',Cy);
setappdata(0,'config_env_G',G);

% objects
rbody = RBody5D(states,dt);

% sim
while rbody.time < tf
   rbody.step;
end

% write
rbody.write;

figure;
plotg(rbody_time,rbody_theta*180/pi);
hold on;
plotg(tout,yout(:,3)*180/pi,'r--');
title('Pitch Angle Response');

figure;
plotg(rbody_time,rbody_vx);
hold on;
plotg(tout,yout(:,2),'r--');
title('Velocity X response');


