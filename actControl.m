% control the actuator cmd to achieve an actuator rate

zeta = 0.707;% (critically damped)
wn = 2*pi*25;% 25Hz actuator bandwidth

% do it block by block and by hand
A = [0 1;-wn^2 -2*zeta*wn];
B = [0;wn^2];

C = [0 1];
D = 0;

[num,den]=ss2tf(A,B,C,D);
G = tf(num,den);

s = tf('s');
C = 35*(1+0.055*s)/s^2;

L = C*G;
T = L/(1+L);

t = [0:0.005:1];
u = ones(length(t),1);
sys = ss(T);
lsim(sys,u,t);
grid on;