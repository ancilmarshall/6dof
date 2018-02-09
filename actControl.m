% control the actuator cmd to achieve an actuator rate
clear all;
close all;

zeta = 0.707;% (critically damped)
wn = 2*pi*25;% 25Hz actuator bandwidth

% do it block by block and by hand
A = [0 1;-wn^2 -2*zeta*wn];
B = [0;wn^2];

C = [1 0];
D = 0;

[num,den]=ss2tf(A,B,C,D);
G = tf(num,den);

s = tf('s');
C = 35*(1+0.055*s)/s^2;

dt = 0.01;
Cd = c2d(C,dt);

L = C*G;
T = L/(1+L);

% test in 6DOF Simulation the a SecondOrderActuator
t0 = 0;
t1 = 1;
t = (t0:dt:t1)';
u = ones(length(t),1);
sys = ss(T);
yout = lsim(sys,u,t);

% Cd =
%  
%   0.021 z - 0.0175
%   ----------------
%    z^2 - 2 z + 1


%%% Configuration

setappdata(0,'config_act_zeta',zeta); % (critically damped)
setappdata(0,'config_act_wn',wn); % actuator bandwidth
setappdata(0,'config_control_b',Cd.Numerator{:});
setappdata(0,'config_control_a',Cd.Denominator{:});

actuatorStates = [0 0]';
act = SecondOrderActuator(actuatorStates,dt);
comp = Controller(dt);

% register producers

% register consumers
act.controller = comp;

while comp.time < t1
    act.step;
    setappdata(0,'data_control_ref',1);
    setappdata(0,'data_control_feedback',getappdata(0,'data_act_OmegaDot'));
    comp.step;
end

% write the output
comp.write;
act.write;



