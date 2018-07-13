clear all;
close all;
load test_butter_filter_data;

t0 = 0;
t1 = 1;
fs = 200;
dt = 1/fs; % same as sim time

% setup a simple low pass filter
fc = 5; % 5Hz = 2*pi*f = 31 rad/s

wn = fc/(fs/2);

order = 2;
[b,a] = butter(order,wn);


setappdata(0,'config_control_a',a);
setappdata(0,'config_control_b',b);

tspan = (t0:dt:t1)';

filter = Filter(dt);

% register producers

% register consumers

for i=1:length(in)
    setappdata(0,'data_filter_input',in(i));
    filter.step;
end

% write the output
filter.write;


figure(1);clf;
hold on;
plot(filter_time,filter_input,'bo');
plot(filter_time,filter_output,'r--');
grid on;
legend('Filter sim');
title('Test of the Filter class');
ylabel('Filter response');
xlabel('Time (sec)');