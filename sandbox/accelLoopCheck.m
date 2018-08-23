% check that the accel dynamics using only ff to control as no steady state
% error
clear all;close all;
global Cx G accelCmd

Cx = 0.35;
G = 9.81;
accelCmd = 4;

tspan = [0:10];
x0 = 0;

[v,t] = ode45(@accelLoopCheck_eom,tspan,x0);
theta = -atan2(accelCmd + Cx*v,G);
accel = -Cx*v - G*tan(theta);

plotg(t,v);
ylim([0 6]);

figure;
plotg(t,theta*180/pi);

figure;
plotg(t,accel);

function vdot = accelLoopCheck_eom(t,v);
global Cx G accelCmd

% calculate control

theta = -atan2(accelCmd + Cx*v,G);

% equations of motion
vdot = -Cx*v - G*tan(theta);

end