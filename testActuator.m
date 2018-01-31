clear all;
%close all;

% test system
% actuatorStates  [Omegax,Omegay,Omegaz,OmegaDotx,OmegaDoty,OmegaDotz]
% rbodyStates = [u,v,w,p,q,r,q0,q1,q2,q3]
t0 = 0;
tf = 0.5;
dt = 0.005; % same as sim time

%%% Configuration

setappdata(0,'config_act_zeta',0.707); % (critically damped)
setappdata(0,'config_act_wn',5*2*pi); % 15Hz actuator bandwidth

tspan = [t0:dt:tf]';
actuatorStates = [1000 0 0 0 0 0]';
[tout,xout] = ode45(@test_actuator_eom,tspan,actuatorStates);

rbodyStates = [0 0 0 3 0 0 0 0 0 0]';
rbody = RBody(rbodyStates,dt);

act = MomentumWheelActuator(actuatorStates,dt);
act.control = [0;0;0];

% register producers
rbody.act = act;

% register consumers

while act.time < tf
    rbody.step;
    act.step;
end

% write the output
act.write;
rbody.write;

figure(1);clf;
subplot(211);
plot(act_time,act_Omegax);
hold on;
plot(tout,xout(:,1),'r--');
grid on;
title('\Omega');
subplot(212)
plot(act_time,act_OmegaDotx);
grid on;
title('\Omega Dot');

%figure(2);clf;
figure(2);clf;
plot(rbody_time,rbody_omega_x);
grid on;
title('Rbody \omega_x');
