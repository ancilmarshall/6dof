clear;
close all;

t0 = 0;
tf = 10;
tspan = [t0;tf];
x0 = [1 2 3 1 2 3 0 0 0 0]';
[tout,xout] = ode45(@test_eom,tspan,x0(1:3));

dt = 0.01; % same as sim time
states = x0;

rbody = RBody(states,dt);

% change to a sim time. Allow multiple rates of running different objects
% and different output rates. Integration rate, output rate. RK can have
% different internal minimum rates also. 
while rbody.time < tf
    rbody.step;
end

% write the output
rbody.write;

figure;
plot(rbody_time,rbody_omega_y,'b-','linewidth',8);
hold on;
plot(tout,xout(:,2),'r','linewidth',2);
grid on;
title('6dof Sim vs Expected');
xlabel('Time (sec)');
ylabel('\omega_y (rad/s)');

