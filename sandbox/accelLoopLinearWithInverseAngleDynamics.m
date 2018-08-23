
close all;
s = tf('s');
Cx = 0.47;
G = 9.81;
wn = 11;
zeta = 0.9;
Gaccel = -G*s/(s+Cx);
Gtheta = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

Gaccel_inv = 1/Gaccel;

G1 = minreal(Gaccel_inv * Gtheta * Gaccel);

%dynamics are basically the theta dynamics
step(G1);

% What does it look like if the Gaccel and Gtheta are combined into
% one system
a = [-Cx -G 0;
      0   0 1;
      0 -wn^2 -2*zeta*wn];
b = [0;0;wn^2];
c = [-Cx -G 0];
d = 0;
sys = ss(a,b,c,d);
G2  = minreal(Gaccel_inv * sys);
hold on;
step(G2);

% so same! 


% What does it look like if the control is linear and the plant is
% nonlinear




function vdot = accelLoopWithAngleDynamics_eom(t,v);
global Cx G accelCmd

% calculate control

theta = -atan2(accelCmd + Cx*v,G);

% equations of motion
vdot = -Cx*v - G*tan(theta);
thetadot = q;
qdot = -wn^2

end

