clear;
close all;

t0 = 0;
t1 = 1;
dt = 0.01; % same as sim time

% setup a simple 2nd order system to set in the filter of the Controller class
s=tf('s');
wn = 11;
zeta = 0.8;
G = wn^2/(s^2+2*zeta*wn*s+wn^2);
[num,den] = pade(dt/2,2);

Gd = c2d(G,dt,'zoh');

%%% Configuration
b = Gd.Numerator{:};
a = Gd.Denominator{:};

setappdata(0,'config_control_a',a);
setappdata(0,'config_control_b',b);

tspan = (t0:dt:t1)';

filter = Filter(dt);

% register producers

% register consumers

while filter.time < t1
    setappdata(0,'data_filter_input',1);
    filter.step;
end

% write the output
filter.write;

[yout,tout]=step(G,t1);


figure(1);clf;
hold on;
plot(filter_time,filter_output,'ro');
plot(tout,yout,'b-');
grid on;
legend('Filter sim','exact with zoh');
title('Test of the Filter class');
ylabel('Filter response');
xlabel('Time (sec)');

