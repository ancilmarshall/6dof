function xdot = dynamics(t,x)

I = eye(3);

xdot = -cross(x,I*x) - x;

end