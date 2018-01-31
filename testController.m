clear all;
%close all;

t0 = 0;
t1 = 1;
dt = 0.01; % same as sim time

% setup a simple 2nd order system to get on the filter in the
% Controller class
s=tf('s');
wn = 11;
zeta = 0.8;
G = wn^2/(s^2+2*zeta*wn*s+wn^2);
[num,den] = pade(dt/2,2);
G_lag = tf(num,den)*G;

Gd = c2d(G,dt,'zoh');

%%% Configuration
b = Gd.Numerator{:};
a = Gd.Denominator{:};

setappdata(0,'config_control_a',a);
setappdata(0,'config_control_b',b);

tspan = (t0:dt:t1)';

comp = Controller(dt);

% register producers

% register consumers

while comp.time < t1
    setappdata(0,'data_control_input',1);
    comp.step;
end

% write the output
comp.write;

[yout,tout]=step(G_lag,t1);


figure(1);clf;
hold on;
plot(control_time,control_output,'ro');
plot(tout,yout,'b-');
grid on;
legend('controller','exact with zoh');