clear all;
%close all;

% test system
% states  [Omegax,Omegay,Omegaz,OmegaDotx,OmegaDoty,OmegaDotz]
t0 = 0;
tf = 5;
dt = 0.01; % same as sim time
tspan = [t0:dt:tf]';
x0 = [1000 0 1000 0 1000 0]';
[tout,xout] = ode45(@test_actuator_eom,tspan,x0);

states = x0;

act = MomentumWheelActuator(states,dt);
act.control = [0;0;0];

% register producers

% register consumers

while act.time < tf
    act.step;
end

% write the output
act.write;

figure(1);clf;
subplot(211);
plot(act_time,[act_Omegax,act_Omegay, act_Omegaz]);
hold on;
plot(tout,xout(:,1),'r--');
grid on;
title('\Omega');
subplot(212)
plot(act_time,[act_OmegaDotx act_OmegaDoty act_OmegaDotz]);
grid on;
title('\Omega Dot');
