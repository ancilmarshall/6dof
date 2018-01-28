% Todo:
% sim object, manager
% input/output for models
% states only for models that implement IntegrationServices (dyanamic
% models) otherwise, static models and don't need dt (just get's time)
% models should have just inputs and outputs (dynamics models adds states)
% output of matricies/vectors should be automatic
% writer should have an interface with the Write required method
% which implements write by default must have the write variable
% instanctiated in the objects constructor.

clear;
close all;

% test system
% states  [u,v,w,p,q,r,q0,q1,q2,q3,Omegax,Omegay,Omegaz]
t0 = 0;
tf = 10;
tspan = [t0;tf];
x0 = [1 2 3 1 2 3 0 0 0 0]';
[tout,xout] = ode45(@test_eom,tspan,x0(1:3));

dt = 0.01; % same as sim time
states = x0;

rbody = RBody(states,dt);
nav = Nav(dt);

% register producers
%nav.registerProducer(rbody);

% register consumers
rbody.registerConsumer(nav);


% change to a sim time. Allow multiple rates of running different objects
% and different output rates. Integration rate, output rate. RK can have
% different internal minimum rates also. 
% while loop could be sequenced by the sim manager
% all objects with integration services must have a step function
% others should have an update function which is the measurement or uses
% outputs of the integration and should come after
% depending on the producers. Or setup binding.
while rbody.time < tf
    rbody.step;
    nav.step;
end

% write the output
% could be sequenced by the sim manager since he should have a list of
% objects
rbody.write;
nav.write;

figure;
plot(rbody_time,rbody_omega_y,'b-','linewidth',8);
hold on;
plot(tout,xout(:,2),'r','linewidth',2);
grid on;
title('6dof Sim vs Expected');
xlabel('Time (sec)');
ylabel('\omega_y (rad/s)');

