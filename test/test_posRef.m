% Current Position ref system. 
% Double integrator with feedback loop cascaded and gains in feedforward
% position
clear all; close all;
global Kp Kd

s = tf('s');

% based on hand calcs closing the loops
wn = 1.5;
zeta = 0.8;
Kd = 2*zeta*wn;
Kp = wn*wn/Kd;

%inner loop
Gi = 1/s;
Li = Kd*Gi;
Ti = minreal(Li/(1+Li));

%outer loop
Go = Ti/s;
Lo = Kp*Go;
To = minreal(Lo/(1+Lo));

% step response
step(To);
grid on;

% what is x,v,u in giving the transfer function methods above? 
% how do we know the internal states? Do the algebra. 
% Using simulink one easy solution. 
% Can also try state space, as well as simualation using ode45
% Best approch is a real sim objective code than can step through all the 
% logic. 
% Simulink is a good substitute for rapid work if you don't have a
% objective simulation code

% try state-spaced
A = [0 1;0 0];
B = [0;1];
K = Kp*Kd;
Kbar = [K Kd];

Acl = (A-B*Kbar);
Bcl = B*K;
Ccl = eye(2);
Dcl = [0;0];

sys = ss(Acl,Bcl,Ccl,Dcl);
figure;
% [Y,T] = step(sys);
% x = Y(:,1);
% v = Y(:,2);
% r = ones(length(x),1);
%u = K*r - K*x - Kd*v;
%plot(T,[x v u]);


% now let's simulate it simply using ode45
tspan = [0 6];
x0 = 0;
v0 = 0.3;
state0 = [x0 v0];
[tout,yout] = ode45(@ref2ndOrder_eom,tspan,state0);
hold on;

x = yout(:,1);
v = yout(:,2);
r = ones(length(tout),1);
u = K*r - K*x - Kd*v;
hold on;
plotg(tout,[x v u]);

legend('xref','vref','aref = u');
xlabel('Time (sec)');
ylabel('Amplitude');
title('Position Ref Step Response');


% how let's perform my 6dof
dt = 0.01;
pref = PositionRef(dt);
pref.x = x0;
pref.v = v0;

while pref.time < 6
   pref.step;
end

pref.write;

hold on;
plot(pref_time,[pref_x pref_v pref_u],'k--');


function xdot = ref2ndOrder_eom(t,state)
global Kp Kd
r = 1;
x = state(1);
v = state(2);

w = Kp*(r-x);
u = Kd*(w-v);

xdot = zeros(2,1);
xdot(1) = v;
xdot(2) = u;

end


