clear;
close all;

tspan = [0;10];
x0 = [1 2 3];

[tout,xout] = ode45(@test_eom,tspan,x0);

dt = 0.01; % same as sim time
states = [1 2 3];

rbody = RBody(states,dt);

while rbody.time < 10
    rbody.step;
end

rbody.write;

figure;
plot(rbody_time,rbody_omega_y,'b-','linewidth',8);
hold on;
plot(tout,xout(:,2),'r','linewidth',2);
grid on;
title('6dof Sim vs Expected');
xlabel('Time (sec)');
ylabel('\omega_y (rad/s)');

