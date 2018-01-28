function derivatives = test_actuator_eom(t,state)

    zeta = 0.707;% (critically damped)
    wn = 25/(2*pi);% 15Hz actuator bandwidth

    % do it block by block and by hand
    A = [0 1;-wn^2 -2*zeta*wn];
    B = [0;wn^2];
    u = 0;
    
    x = state(1:2);
    y = state(3:4);
    z = state(5:6);
    
    xdot = A*x+B*u;
    ydot = A*y+B*u;
    zdot = A*z+B*u;
    
    derivatives = [xdot;ydot;zdot];

end