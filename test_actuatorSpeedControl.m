% control the actuator cmd to achieve an actuator position
% use a digital filter based on a PI compensator
clear;
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
C = 0.55*(s+104)/s;
dt = 0.002; % NOTE: Need to run at high rate b/c actuator BW is high 25Hz
Cd = c2d(C,dt);

% add zero order hold lag
[num,den] = pade(dt/2,2);
G_lag = tf(num,den);
L = C*G*G_lag;
T = L/(1+L);

% test in 6DOF Simulation the a SecondOrderActuator
t0 = 0;
t1 = 0.1;
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
filter = Filter(dt);

% register producers

% register consumers
act.controller = filter;

while filter.time < t1
    act.step;
    feedback = getappdata(0,'data_act_Omega');
    ref = 1;
    error = ref-feedback;
    setappdata(0,'data_filter_input',error);
    filter.step;
end

% write the output
filter.write;
act.write;

figure;
plot(t,yout,'b');
hold on;
plot(act_time,act_Omega,'r');
grid on;
xlim([0 t1]);
ylim([0 1.2]);
legend('Exact solution','Simulation');
xlabel('Time (sec)');
ylabel('\Omega (rad/s)');
title('Actuator Speed Control - Control on \Omega');


