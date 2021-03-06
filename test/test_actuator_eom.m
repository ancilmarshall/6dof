function derivatives = test_actuator_eom(t,state)

    wn = getappdata(0,'config_act_wn');
    zeta = getappdata(0,'config_act_zeta');
    
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